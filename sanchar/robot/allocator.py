"""Decentralized task market: sequential single-item auction with consensus-free awards.

No auctioneer and no leader. Task records are gossiped and merged with a deterministic,
commutative, idempotent rule (a CRDT), so any subset of robots that can hear each other converges
to the same owner for every task, and two network islands that both awarded a task converge to one
owner the moment they reconnect.

Record merge (merge()):
  1. higher `version` wins (versions bump on release / re-open);
  2. same version: more progress wins (done > picked > awarded > open);
  3. same version and status: lower (cost, owner) wins.

Robot behaviour (TaskMarket.tick):
  * idle robots bid on the open task that is cheapest for them (predicted ETA to the pickup,
    obstacle-aware, plus a congestion estimate when an edge model is attached);
  * after the bid window, a robot whose bid is the best it has heard claims the task
    (status=awarded, owner=me, cost=bid); concurrent claims resolve by rule 3;
  * reaching the pickup marks the task `picked`; reaching the drop-off marks it `done`;
  * if the owner's route gets much worse (blocked aisle) it RELEASES the task (version+1, open)
    and everyone re-bids: the PS's "automatically re-assign pickup points" requirement;
  * if an owner falls silent for `task_timeout` ticks (crash, dead zone), any robot re-opens its
    task (version+1). The robot that hears the re-open first does not matter: the merge rule does.
"""
from __future__ import annotations

import random
from msgspec.structs import replace
from typing import Callable

from sanchar.protocol.messages import Bid, TaskAnnounce
from sanchar.robot.planner import static_path
from sanchar.world.roadmap import Roadmap

STATUS_RANK = {"open": 0, "awarded": 1, "picked": 2, "done": 3}


def merge(a: TaskAnnounce | None, b: TaskAnnounce) -> TaskAnnounce:
    if a is None:
        return b
    if a.version != b.version:
        return a if a.version > b.version else b
    ra, rb = STATUS_RANK[a.status], STATUS_RANK[b.status]
    if ra != rb:
        return a if ra > rb else b
    if (a.owner is None) != (b.owner is None):
        return a if a.owner is not None else b
    # total order on everything else, so merge is commutative, associative and idempotent
    ka = (a.cost, -1 if a.owner is None else a.owner, a.dropoff, a.pickup, a.created_tick)
    kb = (b.cost, -1 if b.owner is None else b.owner, b.dropoff, b.pickup, b.created_tick)
    return a if ka <= kb else b


class TaskMarket:
    def __init__(self, rm: Roadmap, robot_id: int, window: int = 2, task_timeout: int = 25,
                 release_factor: float = 1.6, release_slack: int = 6, low_battery: float = 0.2,
                 seed: int = 0):
        self.rm, self.id = rm, robot_id
        self.window, self.task_timeout = window, task_timeout
        self.release_factor, self.release_slack = release_factor, release_slack
        self.low_battery = low_battery
        self.rng = random.Random(seed * 7907 + robot_id)
        self.tasks: dict[str, TaskAnnounce] = {}
        self.bids: dict[str, dict[int, tuple[float, int]]] = {}   # task -> robot -> (cost, version)
        self.first_seen: dict[str, int] = {}
        self.my_task: str | None = None
        self.my_bid: tuple[str, float, int] | None = None          # (task, cost, version)
        self.planned: dict[str, int] = {}                           # task -> ETA at claim time
        self.dirty: set[str] = set()
        self.cost_fn: Callable[[int, int, set[int]], float] | None = None
        self.events: list[str] = []
        self.stats = {"claims": 0, "lost_claims": 0, "releases": 0, "reopens": 0, "done": 0}

    # ---- gossip in ------------------------------------------------------------------------
    def ingest(self, body, t: int) -> None:
        if isinstance(body, TaskAnnounce):
            old = self.tasks.get(body.task_id)
            new = merge(old, body)
            if new is not old:
                self.tasks[body.task_id] = new
                self.dirty.add(body.task_id)
                self.first_seen.setdefault(body.task_id, t)
        elif isinstance(body, Bid):
            rec = self.tasks.get(body.task_id)
            ver = rec.version if rec else 0
            self.bids.setdefault(body.task_id, {})[body.robot_id] = (body.cost, ver)

    def inject(self, task: TaskAnnounce, t: int) -> None:
        """A task arriving from the WMS through this robot (any robot can be the entry point)."""
        self.ingest(task, t)

    # ---- helpers --------------------------------------------------------------------------
    def _cost(self, node: int, target: int, obstacles: set[int]) -> float:
        if target in obstacles:
            return float("inf")
        if self.cost_fn is not None:
            return self.cost_fn(node, target, obstacles)
        path = static_path(self.rm, node, target, obstacles)
        return float(len(path) - 1) if path[-1] == target else float("inf")

    def _set(self, rec: TaskAnnounce) -> None:
        self.tasks[rec.task_id] = rec
        self.dirty.add(rec.task_id)

    def _parking_spot(self, coord) -> int | None:
        """If I idle on a station an active order uses, pick the nearest free spot that is not a
        station, not inside a single-lane corridor and not a corridor mouth."""
        node = coord.node
        busy_stations = {x for r in self.tasks.values() if r.status != "done" for x in (r.pickup, r.dropoff)}
        if self.rm.nodes[node].kind not in ("station", "charger") or node not in busy_stations:
            return None
        mouths = {m for pair in self.rm.mouths.values() for m in pair if m is not None}
        taken = {pv.hello.node for pv in coord.peers.values()} | {pv.hello.claim for pv in coord.peers.values()}
        seen, frontier = {node}, [node]
        while frontier:
            nxt = []
            for u in frontier:
                for v in self.rm.neighbors(u):
                    if v in seen:
                        continue
                    seen.add(v)
                    nxt.append(v)
                    nd = self.rm.nodes[v]
                    if nd.kind == "aisle" and nd.corridor is None and v not in mouths and v not in taken                             and v not in busy_stations and v not in coord.obstacles:
                        return v
            frontier = nxt
        return None


    # ---- per tick -------------------------------------------------------------------------
    def tick(self, t: int, coord, alive_peers: set[int], battery: float,
             chargers: list[int] | None = None) -> list:
        """Advance the market for this robot; returns bodies to broadcast. Drives coord's goal."""
        out: list = []
        self.events = []
        node = coord.node
        obstacles = set(coord.obstacles)

        # 1. do I still own my task?
        if self.my_task is not None:
            rec = self.tasks.get(self.my_task)
            if rec is None or rec.owner != self.id or rec.status in ("open", "done"):
                if rec is not None and rec.status != "done":
                    self.stats["lost_claims"] += 1
                    self.events.append(f"lost {self.my_task} to R{rec.owner}")
                self.my_task = None
                coord.set_goal(None)

        # 2. progress on my task
        if self.my_task is not None:
            rec = self.tasks[self.my_task]
            if rec.status == "awarded" and node == rec.pickup:
                rec = replace(rec, status="picked")
                self._set(rec)
                coord.set_goal(rec.dropoff, rec.task_id, load=True)
                c = self._cost(node, rec.dropoff, obstacles)
                self.planned[rec.task_id] = t + (int(c) if c < float("inf") else 10**6)
                self.events.append(f"picked {rec.task_id}")
            elif rec.status == "picked" and rec.dropoff in obstacles:
                alts = [s for s in self.rm.stations() if s not in obstacles and s != rec.dropoff]
                if alts:
                    alt = min(alts, key=lambda s: (self.rm.dist(rec.dropoff, s), s))
                    rec = replace(rec, dropoff=alt, version=rec.version + 1)
                    self._set(rec)
                    coord.set_goal(alt, rec.task_id, load=True)
                    self.events.append(f"drop station blocked: {rec.task_id} redirected to {alt}")
            elif rec.status == "picked" and node == rec.dropoff:
                rec = replace(rec, status="done")
                self._set(rec)
                self.my_task = None
                self.stats["done"] += 1
                coord.set_goal(None)
                self.events.append(f"done {rec.task_id}")
            elif rec.status == "awarded":
                # 3. blocked aisle: re-assign if my route to the pickup got much worse
                eta = self._cost(node, rec.pickup, obstacles)
                promised = self.planned.get(rec.task_id, t) - t
                if eta == float("inf") or eta > self.release_factor * max(promised, 1) + self.release_slack:
                    rec = replace(rec, version=rec.version + 1, status="open", owner=None, cost=0.0)
                    self._set(rec)
                    self.my_task = None
                    self.stats["releases"] += 1
                    coord.set_goal(None)
                    self.events.append(f"released {rec.task_id}: route blocked (eta {eta:.0f} vs {promised})")

        # 4. re-open tasks whose owner went silent
        for tid, rec in list(self.tasks.items()):
            if rec.owner in alive_peers:
                self.first_seen.pop(f"silent:{tid}:{rec.version}", None)
            if rec.status in ("awarded", "picked") and rec.owner is not None and rec.owner != self.id \
                    and rec.owner not in alive_peers:
                seen = self.first_seen.get(f"silent:{tid}:{rec.version}")
                if seen is None:
                    self.first_seen[f"silent:{tid}:{rec.version}"] = t
                elif t - seen >= self.task_timeout:
                    new = replace(rec, version=rec.version + 1, status="open", owner=None, cost=0.0)
                    self._set(new)
                    self.stats["reopens"] += 1
                    self.events.append(f"re-opened {tid}: owner R{rec.owner} silent")

        # 5. idle: bid, then claim if my bid is the best I have heard after the window
        if self.my_task is None:
            if battery < self.low_battery and chargers:
                target = min(chargers, key=lambda c: self.rm.dist(node, c))
                if coord.goal != target:
                    coord.set_goal(target, "charge", load=False)
                self.my_bid = None
            else:
                opens = [r for r in self.tasks.values() if r.status == "open"]
                if coord.task_id == "charge" and battery >= 0.9:
                    coord.set_goal(None)
                best = None
                for r in opens:
                    c = self._cost(node, r.pickup, obstacles)
                    if c < float("inf") and (best is None or (c, r.task_id) < (best[1], best[0].task_id)):
                        best = (r, c)
                if best is None and coord.goal is None:
                    park = self._parking_spot(coord)
                    if park is not None:
                        coord.set_goal(park, "park", load=False)
                        self.events.append(f"parking off station at {park}")
                if best is not None:
                    r, c = best
                    self.my_bid = (r.task_id, c, r.version)
                    self.bids.setdefault(r.task_id, {})[self.id] = (c, r.version)
                    out.append(Bid(task_id=r.task_id, robot_id=self.id, cost=c, tick=t))
                    seen = self.first_seen.setdefault(r.task_id, t)
                    if t - seen >= self.window:
                        rivals = {rid: cv for rid, cv in self.bids.get(r.task_id, {}).items()
                                  if cv[1] == r.version and (rid == self.id or rid in alive_peers)}
                        if min(rivals.items(), key=lambda kv: (kv[1][0], kv[0]))[0] == self.id:
                            rec = replace(r, status="awarded", owner=self.id, cost=c)
                            self._set(rec)
                            self.my_task = r.task_id
                            self.my_bid = None
                            self.planned[r.task_id] = t + int(c)
                            coord.set_goal(r.pickup, r.task_id, load=False)
                            self.stats["claims"] += 1
                            self.events.append(f"claimed {r.task_id} (bid {c:.0f})")

        # 6. gossip: changed records now, plus one random record for anti-entropy
        send = set(self.dirty)
        if self.tasks and self.rng.random() < 0.5:
            send.add(self.rng.choice(list(self.tasks)))
        out.extend(self.tasks[tid] for tid in sorted(send))
        self.dirty.clear()
        return out

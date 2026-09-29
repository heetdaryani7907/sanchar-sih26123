"""Live-order simulation: tasks arrive over time, robots allocate them among themselves.

This is the full PS loop: decentralized communication + conflict resolution + task allocation and
re-routing, with battery drain and charging. The WMS only *injects* orders (to two random robots,
for redundancy); it never assigns them. Everything else happens robot-to-robot over the lossy bus.

Baseline: stop-and-wait motion + a centralized greedy dispatcher with perfect information (the
oldest open order goes to the nearest idle robot), i.e. a conventional fleet manager.
"""
from __future__ import annotations

import random
import time
from dataclasses import dataclass, field

from baseline.stop_and_wait import StopAndWaitPolicy
from sanchar.protocol.messages import TaskAnnounce
from sanchar.robot.allocator import TaskMarket
from sanchar.sim.engine import Chaos, World, apply_moves
from sanchar.sim.policies import SancharPolicy
from sanchar.world.roadmap import Roadmap


@dataclass
class Order:
    task_id: str
    release: int
    pickup: int
    dropoff: int


@dataclass
class MarketScenario:
    name: str
    layout: list[str]
    starts: list[int]
    orders: list[Order]
    chaos: list[Chaos] = field(default_factory=list)
    loss: float = 0.0
    max_ticks: int = 1500
    drain_per_move: float = 0.0015
    drain_per_tick: float = 0.0002
    charge_per_tick: float = 0.02
    start_battery: tuple[float, float] = (0.6, 1.0)
    description: str = ""


class SancharMarketPolicy(SancharPolicy):
    name = "sanchar"

    def reset(self, world: World, seed: int) -> None:
        super().reset(world, seed)
        self.markets = {r: TaskMarket(world.rm, r, seed=seed) for r in self.agents}
        self.market_events: list[tuple[int, int, str]] = []

    def inject(self, order: Order, world: World) -> None:
        rec = TaskAnnounce(task_id=order.task_id, pickup=order.pickup, dropoff=order.dropoff,
                           created_tick=order.release, origin=-1)
        alive = sorted(world.alive)
        for rid in self.rng.sample(alive, min(2, len(alive))):
            self.markets[rid].inject(rec, world.tick)

    def _deliver(self, dst: int, body, rnd: int) -> None:
        if isinstance(body, (TaskAnnounce,)) or type(body).__name__ == "Bid":
            self.markets[dst].ingest(body, rnd // 2)
        else:
            self.agents[dst].ingest(body, rnd)

    def _pre_claim(self, rid: int, world: World) -> list:
        a = self.agents[rid]
        a.set_battery(world.battery[rid])
        r = 2 * world.tick
        alive = {p for p, pv in a.peers.items() if r - pv.heard_round <= 6}
        out = self.markets[rid].tick(world.tick, a, alive, world.battery[rid], world.rm.chargers())
        for e in self.markets[rid].events:
            self.market_events.append((world.tick, rid, e))
        return out

    def done_tasks(self) -> set[str]:
        done = set()
        for m in self.markets.values():
            done |= {tid for tid, rec in m.tasks.items() if rec.status == "done"}
        return done

    def stats(self) -> dict:
        st = super().stats()
        for k in ("claims", "lost_claims", "releases", "reopens"):
            st[f"market_{k}"] = sum(m.stats[k] for m in self.markets.values())
        return st


class CentralDispatchBaseline(StopAndWaitPolicy):
    name = "stop_and_wait+central_dispatch"

    def reset(self, world: World, seed: int) -> None:
        super().reset(world, seed)
        self.queue: list[Order] = []
        self.job: dict[int, tuple[Order, str] | None] = {r: None for r in world.pos}
        self.done: set[str] = set()

    def inject(self, order: Order, world: World) -> None:
        self.queue.append(order)

    def step(self, world: World) -> dict[int, int]:
        for rid in sorted(world.alive):
            job = self.job[rid]
            if job is not None:
                order, phase = job
                if phase == "pickup" and world.pos[rid] == order.pickup:
                    self.job[rid] = (order, "drop")
                    self.set_goal(rid, order.dropoff, order.task_id, True)
                elif phase == "drop" and world.pos[rid] == order.dropoff:
                    self.done.add(order.task_id)
                    self.job[rid] = None
                    self.set_goal(rid, None, None, False)
            if self.job[rid] is None and world.battery[rid] < 0.2 and world.rm.chargers():
                c = min(world.rm.chargers(), key=lambda n: self.rm.dist(world.pos[rid], n))
                if self.goal[rid] != c:
                    self.set_goal(rid, c, "charge", False)
            elif self.job[rid] is None and self.goal[rid] is not None and world.battery[rid] >= 0.9:
                self.set_goal(rid, None, None, False)
        idle = [r for r in sorted(world.alive) if self.job[r] is None and world.battery[r] >= 0.2]
        while self.queue and idle:
            order = self.queue.pop(0)
            rid = min(idle, key=lambda r: (self.rm.dist(world.pos[r], order.pickup), r))
            idle.remove(rid)
            self.job[rid] = (order, "pickup")
            self.set_goal(rid, order.pickup, order.task_id, False)
        # crashed robots' jobs go back to the queue (the central manager sees the crash instantly)
        for rid, job in self.job.items():
            if job is not None and rid not in world.alive:
                self.queue.insert(0, job[0])
                self.job[rid] = None
        return super().step(world)

    def done_tasks(self) -> set[str]:
        return self.done


def run_market(scn: MarketScenario, policy, seed: int = 0, on_tick=None) -> dict:
    rm = Roadmap.from_ascii(scn.layout)
    n = len(scn.starts)
    rng = random.Random(seed * 31 + 7)
    world = World(rm, pos=dict(enumerate(scn.starts)), alive=set(range(n)), present=set(range(n)),
                  loss=scn.loss)
    world.battery = {r: rng.uniform(*scn.start_battery) for r in range(n)}
    policy.reset(world, seed)
    pending = sorted(scn.orders, key=lambda o: o.release)
    release = {o.task_id: o.release for o in scn.orders}
    done_at: dict[str, int] = {}
    chaos = sorted(scn.chaos, key=lambda c: c.tick)
    counters = [0, 0, 0, 0]
    makespan = None
    wall0 = time.perf_counter()
    for t in range(scn.max_ticks):
        world.tick = t
        while chaos and chaos[0].tick <= t:
            c = chaos.pop(0)
            if c.kind == "block":
                world.obstacles |= set(c.arg)
            elif c.kind == "unblock":
                world.obstacles -= set(c.arg)
            elif c.kind == "crash":
                world.alive.discard(c.arg)
            elif c.kind == "remove":                  # technician wheels a dead robot off the floor
                world.alive.discard(c.arg)
                world.present.discard(c.arg)
            elif c.kind == "deadzone":
                world.dead_zone |= set(c.arg)
            elif c.kind == "clear_deadzone":
                world.dead_zone -= set(c.arg)
            elif c.kind == "partition":
                world.partition = dict(c.arg)
            elif c.kind == "heal":
                world.partition = {}
            elif c.kind == "loss":
                world.loss = float(c.arg)
        while pending and pending[0].release <= t:
            policy.inject(pending.pop(0), world)
        prev = dict(world.pos)
        moves = policy.step(world)
        res = apply_moves(world, moves)
        counters = [a + b for a, b in zip(counters, res)]
        chargers = set(rm.chargers())
        for r in world.alive:
            moved = world.pos[r] != prev[r]
            b = world.battery[r] - scn.drain_per_tick - (scn.drain_per_move if moved else 0.0)
            if not moved and world.pos[r] in chargers:
                b += scn.charge_per_tick
            world.battery[r] = max(0.0, min(1.0, b))
        for tid in policy.done_tasks():
            done_at.setdefault(tid, t + 1)
        if on_tick is not None:
            on_tick(world, {"t": t, "done": len(done_at)})
        if not pending and len(done_at) == len(scn.orders):
            makespan = t + 1
            break
    lat = [done_at[k] - release[k] for k in done_at]
    total = len(scn.orders)
    return {
        "scenario": scn.name, "policy": policy.name, "seed": seed, "robots": n,
        "makespan": makespan if makespan is not None else scn.max_ticks,
        "finished": makespan is not None, "tasks_done": len(done_at), "tasks_total": total,
        "mean_latency": (sum(lat) + (total - len(lat)) * scn.max_ticks) / total,
        "collisions": counters[0], "swaps": counters[1], "obstacle_hits": counters[2],
        "teleports": counters[3], "min_battery": min(world.battery.values()),
        "wall_s": time.perf_counter() - wall0, **policy.stats(),
    }

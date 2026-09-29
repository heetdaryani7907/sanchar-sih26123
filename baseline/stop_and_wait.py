"""Baseline A: traditional stop-and-wait with centralized zone control (the PS's comparison).

Deliberately generous so the comparison is fair, not a strawman:
  * Perfect, lossless information: a central traffic manager sees every robot every tick.
  * Directional corridor zones derived from ground truth every tick: robots heading the SAME way
    may share a single-lane corridor (convoy); the opposite direction waits until it empties.
    Classic AGV zone control allows one vehicle per zone; this version is stronger.
  * Clear-the-mouth rule: a robot waiting to enter a corridor never blocks the exit of robots
    coming out; it steps aside immediately (standard waiting-point practice in AGV layouts).
  * FCFS service order (longest-waiting robot first), so nobody is starved.
  * Randomized timeout backoff: a robot stuck for `stuck_limit` ticks steps to a free neighbour
    (off the blocker's path when possible), holds 2-6 ticks, then recomputes its path.
  * Idle nudging: an idle robot that blocks a working robot is moved off its path by the central
    manager (live-order mode, where robots stay on the floor between jobs).
  * Same physics rule as SANCHAR: never enter a node that is occupied at the start of the tick.
  * Wi-Fi dead zones: a robot inside one cannot hear the fleet manager, so it safety-stops (the
    standard AMR behaviour on link loss); the manager keeps planning everyone else around it.

What it does NOT do (by definition of stop-and-wait): no re-routing around robots or blocked
aisles, no negotiation, no pushing. A robot follows its shortest static path and stops when the
next node is occupied, reserved, blocked, or behind opposing corridor traffic.
"""
from __future__ import annotations

import random

from sanchar.robot.planner import static_path
from sanchar.sim.engine import World


def lane_zones(rm) -> tuple[dict[int, list[int]], dict[int, int], dict[int, tuple[int | None, int | None]]]:
    """Traffic-control zones = maximal single-lane stretches between passing places.

    A node is 'lane' if it is an aisle node with at most two non-pocket neighbours; a lay-by pocket
    does not split the lane (classic zone control does not use pockets dynamically). Returns
    ordered chains, node->zone, and each zone's (end-A mouth, end-B mouth) outside nodes."""
    def lane(n: int) -> bool:
        return rm.nodes[n].kind == "aisle" and sum(1 for m in rm.neighbors(n) if not rm.is_turnout(m)) <= 2
    lanes = {n for n in rm.nodes if lane(n)}
    zones, zone_of, mouths = {}, {}, {}
    for n0 in sorted(lanes):
        if n0 in zone_of:
            continue
        comp, stack = {n0}, [n0]
        while stack:
            u = stack.pop()
            for v in rm.neighbors(u):
                if v in lanes and v not in comp:
                    comp.add(v)
                    stack.append(v)
        ends = [u for u in comp if sum(1 for v in rm.neighbors(u) if v in comp) <= 1]
        start = min(ends) if ends else min(comp)
        chain, prev, cur = [start], None, start
        while True:
            nxt = [v for v in rm.neighbors(cur) if v in comp and v != prev and v not in chain]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            chain.append(cur)
        zid = len(zones)
        zones[zid] = chain
        for u in comp:
            zone_of[u] = zid
        outside = lambda u: sorted(v for v in rm.neighbors(u) if v not in comp and not rm.is_turnout(v))
        oa, ob = outside(chain[0]), outside(chain[-1])
        a = oa[0] if oa else None
        b = (ob[-1] if len(chain) == 1 and len(ob) > 1 else (ob[0] if ob else None))
        mouths[zid] = (a, b)
    return zones, zone_of, mouths


class StopAndWaitPolicy:
    name = "stop_and_wait"

    def __init__(self, stuck_limit: int = 5):
        self.stuck_limit = stuck_limit

    def reset(self, world: World, seed: int) -> None:
        self.world = world
        self.rm = world.rm
        self.rng = random.Random(seed * 7919 + 1)
        self.goal: dict[int, int | None] = {r: None for r in world.pos}
        self.path: dict[int, list[int]] = {r: [] for r in world.pos}
        self.wait = {r: 0 for r in world.pos}
        self.hold = {r: 0 for r in world.pos}
        self.zones = self.rm.corridors
        self.zone_of = {n: c for c, ns in self.zones.items() for n in ns}
        self.mouths = self.rm.mouths
        self.chain = {z: set(ns) for z, ns in self.zones.items()}
        self.idx = {z: {n: i for i, n in enumerate(ns)} for z, ns in self.zones.items()}
        self.backoffs = 0
        self.stop_ticks = 0

    def set_goal(self, rid, goal, task_id, load) -> None:
        self.goal[rid] = goal
        self.path[rid] = static_path(self.rm, self.world.pos[rid], goal) if goal is not None else []

    def remove(self, rid, world) -> None:
        pass

    def _dir(self, z: int, here: int, nxt: int) -> int:
        """+1 = moving from zone end A towards end B, -1 = B towards A, 0 = into a pocket."""
        if self.rm.is_turnout(nxt) or self.rm.is_turnout(here):
            return 0
        idx, (a, b) = self.idx[z], self.mouths[z]
        if here in idx and nxt in idx:
            return 1 if idx[nxt] > idx[here] else -1
        if here not in idx:                       # entering
            return 1 if here == a else -1
        if len(idx) == 1:                         # exiting a one-node zone
            return -1 if nxt == a else 1
        return 1 if idx[here] == len(idx) - 1 else -1

    def _heading(self, rid: int, z: int) -> int:
        """Direction of a robot inside zone z (0 = stopped or parked: blocks both directions)."""
        here = self.world.pos[rid]
        p = self.path.get(rid) or []
        if self.hold[rid] == 0 and len(p) > 1 and p[0] == here:
            return self._dir(z, here, p[1])
        return 0

    def _opposing(self, rid: int, here: int, nxt: int) -> list[int]:
        """Robots inside the zone I am about to enter that head against me (or are stopped)."""
        z = self.zone_of.get(nxt)
        if z is None or self.zone_of.get(here) == z:
            return []
        d = self._dir(z, here, nxt)
        inside = [r for r in self.world.present if r != rid and self.world.pos[r] in self.chain[z]]
        opp = [r for r in inside if self._heading(r, z) != d or d == 0]
        opp += [r for r, d2 in self._entries.get(z, []) if r != rid and (d2 != d or d == 0) and r not in opp]
        return opp

    def _backoff(self, rid, here, blocker, start_occ, reserved, moves) -> bool:
        free = [m for m in self.rm.neighbors(here)
                if m not in start_occ and m not in reserved and m not in self.world.obstacles
                and not self._opposing(rid, here, m)]
        if not free:
            return False
        bpath = set(self.path.get(blocker, [])) if blocker is not None else set()
        off = [m for m in free if m not in bpath]
        m = self.rng.choice(off or free)
        moves[rid] = m
        reserved.add(m)
        z = self.zone_of.get(m)
        if z is not None and self.zone_of.get(here) != z:
            self._entries.setdefault(z, []).append((rid, 0))
        self.hold[rid] = self.rng.randint(2, 6)
        self.path[rid] = static_path(self.rm, m, self.goal[rid])
        self.wait[rid] = 0
        self.backoffs += 1
        return True

    def step(self, world: World) -> dict[int, int]:
        pos = world.pos
        start_occ = {pos[r] for r in world.present}
        reserved: set[int] = set()
        moves: dict[int, int] = {}
        self._entries: dict[int, list[tuple[int, int]]] = {}   # zone -> [(rid, dir)] entered this tick
        # idle nudging: the central manager moves idle robots off working robots' next nodes
        idle = {r for r in world.alive if self.goal[r] is None or pos[r] == self.goal[r]}
        wanted = {}
        for r in world.alive:
            p = self.path[r]
            if r not in idle and len(p) > 1 and p[0] == pos[r]:
                wanted[p[1]] = r
        for r in sorted(idle):
            w = wanted.get(pos[r])
            if w is None:
                continue
            wpath = set(self.path[w])
            free = [m for m in self.rm.neighbors(pos[r]) if m not in start_occ and m not in reserved
                    and m not in world.obstacles and not self._opposing(r, pos[r], m)]
            free.sort(key=lambda m: (m in wpath, not self.rm.is_turnout(m), m))
            if free:
                moves[r] = free[0]
                reserved.add(free[0])
                self.nudges = getattr(self, "nudges", 0) + 1
        for rid in sorted(world.alive, key=lambda r: (-self.wait[r], r)):
            here, goal = pos[rid], self.goal[rid]
            if rid in moves:
                continue
            if here in world.dead_zone:              # no link to the fleet manager: safety stop
                moves[rid] = here
                reserved.add(here)
                continue
            if goal is None or here == goal:
                moves[rid] = here
                reserved.add(here)
                continue
            if self.hold[rid] > 0:                   # randomized backoff hold
                self.hold[rid] -= 1
                moves[rid] = here
                reserved.add(here)
                continue
            path = self.path[rid]
            if not path or path[0] != here:          # displaced by a backoff: recompute
                path = self.path[rid] = static_path(self.rm, here, goal)
            if len(path) < 2:
                moves[rid] = here
                reserved.add(here)
                continue
            nxt = path[1]
            opposing = self._opposing(rid, here, nxt)
            if (nxt not in start_occ and nxt not in reserved and nxt not in world.obstacles
                    and not opposing):
                moves[rid] = nxt
                reserved.add(nxt)
                z = self.zone_of.get(nxt)
                if z is not None and self.zone_of.get(here) != z:
                    self._entries.setdefault(z, []).append((rid, self._dir(z, here, nxt)))
                self.path[rid] = path[1:]
                self.wait[rid] = 0
                continue
            moves[rid] = here
            self.wait[rid] += 1
            self.stop_ticks += 1
            blocker = next((r for r in world.present if pos[r] == nxt), None)
            # clear-the-mouth: opposing traffic will exit through my node, step aside now
            z = self.zone_of.get(nxt)
            exiting_through_me = z is not None and any(self._heading(r, z) != 0 for r in opposing)
            if (exiting_through_me or self.wait[rid] >= self.stuck_limit) and \
                    self._backoff(rid, here, blocker if blocker is not None else (opposing[0] if opposing else None),
                                  start_occ, reserved, moves):
                continue
            reserved.add(here)
        return moves

    def stats(self) -> dict:
        return {"backoffs": self.backoffs, "stop_ticks": self.stop_ticks}

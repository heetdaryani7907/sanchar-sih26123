"""In-process fleet simulation engine (deterministic, seedable).

The engine owns physics and ground truth only: robot positions, static obstacles, crashed robots,
radio conditions (loss, dead zones, partitions) and task progress. It never plans. Policies decide
moves; the engine checks them and counts collisions, so a policy cannot hide a collision.

Policies:
  * SancharPolicy (sanchar/sim/policies.py): one independent Coordinator per robot. The ONLY
    channel between robots is the signed, lossy broadcast bus modelled here.
  * StopAndWaitPolicy (baseline/stop_and_wait.py): traditional centralized zone control.

One tick = one node hop (1 m roadmap spacing, 1 tick = 1 s, so 1 m/s AMR cruise speed).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from sanchar.world.roadmap import Roadmap


@dataclass
class Task:
    task_id: str
    pickup: int
    dropoff: int


@dataclass
class Chaos:
    tick: int
    kind: str          # block | unblock | crash | remove | deadzone | clear_deadzone | partition | heal | loss
    arg: object = None


@dataclass
class Scenario:
    name: str
    layout: list[str]
    starts: list[int]
    tasks: list[list[Task]]            # fixed per-robot task sequence (isolates coordination)
    chaos: list[Chaos] = field(default_factory=list)
    loss: float = 0.0
    max_ticks: int = 1500
    vanish_when_done: bool = True      # robot leaves the floor after its last task
    description: str = ""


@dataclass
class World:
    rm: Roadmap
    tick: int = 0
    pos: dict[int, int] = field(default_factory=dict)
    alive: set[int] = field(default_factory=set)       # robots that compute and move
    present: set[int] = field(default_factory=set)     # robots physically on the floor
    obstacles: set[int] = field(default_factory=set)
    dead_zone: set[int] = field(default_factory=set)
    partition: dict[int, int] = field(default_factory=dict)
    loss: float = 0.0
    p2p_range: int = 3               # direct robot-to-robot link range (grid hops), survives Wi-Fi loss

    def can_talk(self, a: int, na: int, b: int, nb: int) -> bool:
        """Long-range links need infrastructure (no dead zone, same partition). Robots within
        p2p_range of each other always have a direct short-range link (UWB / 802.11 ad-hoc /
        ESP-NOW): exactly the robot-to-robot path the PS asks for."""
        (xa, ya), (xb, yb) = self.rm.pos(na), self.rm.pos(nb)
        if abs(xa - xb) + abs(ya - yb) <= self.p2p_range:
            return True
        if na in self.dead_zone or nb in self.dead_zone:
            return False
        return self.partition.get(a, 0) == self.partition.get(b, 0)

    def sensed_obstacles(self, rid: int) -> list[int]:
        """Onboard proximity sensing, radius 1: blocked neighbour cells and crashed robots."""
        me = self.pos[rid]
        crashed = {self.pos[r] for r in self.present - self.alive}
        return [m for m in self.rm.neighbors(me) if m in self.obstacles or m in crashed]

    def sensed_robots(self, rid: int, radius: int = 2) -> list[tuple[int, int]]:
        """Line-of-sight sensing along the 4 grid directions (LiDAR + fiducial id), radius hops."""
        me = self.pos[rid]
        x0, y0 = self.rm.pos(me)
        where = {self.pos[r]: r for r in self.present if r != rid}
        by_xy = {(nd.x, nd.y): nid for nid, nd in self.rm.nodes.items()}
        seen = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for k in range(1, radius + 1):
                nid = by_xy.get((x0 + dx * k, y0 + dy * k))
                if nid is None:
                    break
                if nid in where:
                    seen.append((where[nid], nid))
                    break
        return seen


def apply_moves(world: World, moves: dict[int, int]) -> tuple[int, int, int, int]:
    """Apply policy moves with physics checks. Returns (collisions, swaps, obstacle_hits, teleports).
    Illegal jumps are refused; collisions are counted, never hidden."""
    rm = world.rm
    prev = dict(world.pos)
    new = dict(prev)
    teleports = obstacle_hits = collisions = swaps = 0
    for rid in world.alive:
        node = moves.get(rid, prev[rid])
        if node != prev[rid] and node not in rm.neighbors(prev[rid]):
            teleports += 1
            node = prev[rid]
        if node in world.obstacles and node != prev[rid]:     # driving INTO an obstacle
            obstacle_hits += 1
        new[rid] = node
    occ: dict[int, int] = {}
    for rid in world.present:
        if new[rid] in occ:
            collisions += 1
        occ[new[rid]] = rid
    pos_of = {prev[r]: r for r in world.present}
    for a in (r for r in world.alive if new[r] != prev[r]):
        b = pos_of.get(new[a])
        if b is not None and b != a and new[b] == prev[a] and a < b:
            swaps += 1
    world.pos = new
    return collisions, swaps, obstacle_hits, teleports


class Policy(Protocol):
    name: str

    def reset(self, world: World, seed: int) -> None: ...
    def set_goal(self, rid: int, goal: int | None, task_id: str | None, load: bool) -> None: ...
    def step(self, world: World) -> dict[int, int]: ...
    def remove(self, rid: int, world: World) -> None: ...
    def stats(self) -> dict: ...


def run(scn: Scenario, policy: Policy, seed: int = 0,
        on_tick: Callable[[World, dict], None] | None = None) -> dict:
    rm = Roadmap.from_ascii(scn.layout)
    n = len(scn.starts)
    world = World(rm, pos=dict(enumerate(scn.starts)), alive=set(range(n)), present=set(range(n)),
                  loss=scn.loss)
    policy.reset(world, seed)
    idx, phase = [0] * n, ["pickup"] * n
    completion: list[int] = []
    task_done_tick: dict[str, int] = {}

    def goal_of(rid: int) -> int | None:
        if phase[rid] == "done":
            return None
        task = scn.tasks[rid][idx[rid]]
        return task.pickup if phase[rid] == "pickup" else task.dropoff

    def assign(rid: int, t: int) -> None:
        while True:
            if idx[rid] >= len(scn.tasks[rid]):
                phase[rid] = "done"
                policy.set_goal(rid, None, None, False)
                if scn.vanish_when_done and rid in world.present:
                    policy.remove(rid, world)
                    world.alive.discard(rid)
                    world.present.discard(rid)
                return
            task = scn.tasks[rid][idx[rid]]
            g = goal_of(rid)
            if world.pos[rid] != g:
                policy.set_goal(rid, g, task.task_id, phase[rid] == "drop")
                return
            if phase[rid] == "pickup":
                phase[rid] = "drop"
            else:
                completion.append(t)
                task_done_tick[task.task_id] = t
                idx[rid] += 1
                phase[rid] = "pickup"

    for rid in range(n):
        assign(rid, 0)

    chaos = sorted(scn.chaos, key=lambda c: c.tick)
    collisions, swaps, obstacle_hits, teleports = 0, 0, 0, 0
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
        moves = policy.step(world)
        c, sw, oh, tp = apply_moves(world, moves)
        collisions, swaps, obstacle_hits, teleports = collisions + c, swaps + sw, obstacle_hits + oh, teleports + tp
        for rid in list(world.alive):
            if phase[rid] != "done" and world.pos[rid] == goal_of(rid):
                assign(rid, t + 1)
        if on_tick is not None:
            on_tick(world, {"t": t, "phase": list(phase), "completion": len(completion)})
        if all(p == "done" or r not in world.alive for r, p in enumerate(phase)):
            makespan = t + 1
            break
    total_tasks = sum(len(ts) for ts in scn.tasks)
    return {
        "scenario": scn.name, "policy": policy.name, "seed": seed, "robots": n,
        "makespan": makespan if makespan is not None else scn.max_ticks,
        "finished": makespan is not None and len(completion) == total_tasks,
        "tasks_done": len(completion), "tasks_total": total_tasks,
        "sum_completion": sum(completion) + (total_tasks - len(completion)) * scn.max_ticks,
        "collisions": collisions, "swaps": swaps, "obstacle_hits": obstacle_hits,
        "teleports": teleports, "wall_s": time.perf_counter() - wall0,
        **policy.stats(),
    }

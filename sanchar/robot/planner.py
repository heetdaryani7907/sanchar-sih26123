"""Guide-path planner: windowed space-time A* over the roadmap.

Each robot plans ONLY for itself. Peers' broadcast Intents (higher priority than me) are entered
into a ReservationTable as moving obstacles for the next H ticks; beyond the horizon the path is
extended statically (obstacle-aware BFS). The resulting guide path is (a) broadcast as my Intent
and (b) the *preference* for the one-step PIBT decision in coordinator.py. It is never a
commitment: the coordinator's safety check decides each actual move.

Conventions: one tick = one node hop or one wait, both cost 1. path[i] is my node at t0+i.
"""
from __future__ import annotations

import heapq
from collections import deque
from functools import lru_cache
from typing import Iterable

from sanchar.world.roadmap import Roadmap


class ReservationTable:
    """Space-time reservations of higher-priority peers. Tail-blocks the last known node so a
    robot that stops broadcasting a plan is still treated as parked, not as vanished."""

    def __init__(self, horizon: int):
        self.horizon = horizon
        self.node_at: dict[int, set[int]] = {}          # t -> nodes occupied
        self.move_at: dict[int, set[tuple[int, int]]] = {}  # t -> (from, to) moves happening t->t+1

    def add_path(self, nodes: list[int], t0: int, tail: bool = True) -> None:
        if not nodes:
            return
        for i, n in enumerate(nodes):
            self.node_at.setdefault(t0 + i, set()).add(n)
            if i + 1 < len(nodes):
                self.move_at.setdefault(t0 + i, set()).add((n, nodes[i + 1]))
        if not tail:
            return
        last = nodes[-1]
        for t in range(t0 + len(nodes), t0 + self.horizon + 2):   # tail block
            self.node_at.setdefault(t, set()).add(last)

    def node_blocked(self, n: int, t: int) -> bool:
        return n in self.node_at.get(t, ())

    def swap_blocked(self, u: int, v: int, t: int) -> bool:
        """Moving u->v during t->t+1 while a peer moves v->u is a head-on swap: forbidden."""
        return (v, u) in self.move_at.get(t, ())


@lru_cache(maxsize=4096)
def _dist_map_avoiding(rm: Roadmap, goal: int, blocked: frozenset[int]) -> dict[int, int]:
    d = {goal: 0}
    q = deque([goal])
    while q:
        u = q.popleft()
        for v in rm.neighbors(u):
            if v not in d and v not in blocked:
                d[v] = d[u] + 1
                q.append(v)
    return d


def static_path(rm: Roadmap, start: int, goal: int, blocked: Iterable[int] = ()) -> list[int]:
    """Shortest path ignoring time (greedy descent on obstacle-aware distance map)."""
    bl = frozenset(blocked) - {start, goal}
    d = _dist_map_avoiding(rm, goal, bl)
    if start not in d:
        return [start]
    path = [start]
    cur = start
    while cur != goal:
        cur = min((m for m in rm.neighbors(cur) if m in d), key=lambda m: (d[m], m))
        path.append(cur)
    return path


def plan_guide_path(rm: Roadmap, start: int, goal: int, res: ReservationTable, t0: int,
                    horizon: int, blocked: Iterable[int] = ()) -> list[int]:
    """Windowed space-time A*. Returns path with path[0] == start at tick t0 and path[-1] == goal
    (or the best reachable prefix + static extension). Never raises."""
    bl = frozenset(blocked) - {start, goal}
    h = _dist_map_avoiding(rm, goal, bl)
    if start == goal:
        return [start]
    if start not in h:                      # goal unreachable given obstacles
        return [start]
    t_end = t0 + horizon
    # state: (f, g, node, t) ; parent map keyed by (node, t)
    start_key = (start, t0)
    g_cost = {start_key: 0}
    parent: dict[tuple[int, int], tuple[int, int] | None] = {start_key: None}
    pq = [(h[start], 0, start, t0)]
    best_key, best_h = start_key, h[start]
    while pq:
        f, g, n, t = heapq.heappop(pq)
        if g_cost.get((n, t), 10**9) < g:
            continue
        if h[n] < best_h or (h[n] == best_h and t > best_key[1] and n != best_key[0]):
            best_key, best_h = (n, t), h[n]
        if n == goal:
            best_key = (n, t)
            break
        if t >= t_end:
            continue
        for m in (*rm.neighbors(n), n):     # move or wait
            if m in bl or res.node_blocked(m, t + 1):
                continue
            if m != n and res.swap_blocked(n, m, t):
                continue
            key = (m, t + 1)
            ng = g + 1
            if ng < g_cost.get(key, 10**9):
                g_cost[key] = ng
                parent[key] = (n, t)
                heapq.heappush(pq, (ng + h[m], ng, m, t + 1))
    # reconstruct to best_key
    path: list[int] = []
    k: tuple[int, int] | None = best_key
    while k is not None:
        path.append(k[0])
        k = parent[k]
    path.reverse()
    if path[-1] != goal:                    # extend statically beyond the window
        ext = static_path(rm, path[-1], goal, bl)
        path.extend(ext[1:])
    return path


def preference_order(rm: Roadmap, node: int, guide_path: list[int], goal: int,
                     blocked: Iterable[int] = (), wait_first: bool = False) -> list[int]:
    """PIBT candidate order: guide-path next node first, then other neighbours by distance to
    goal (turnouts preferred among equals, they are where yielders should go), then stay put."""
    bl = frozenset(blocked)
    d = _dist_map_avoiding(rm, goal, bl - {node, goal})
    nxt = guide_path[1] if len(guide_path) > 1 and guide_path[0] == node else None
    if wait_first and nxt == node:        # planner chose to wait: honour it (config: honour_planned_wait)
        nxt = None
        cands = [m for m in rm.neighbors(node) if m not in bl]
        return [node] + sorted(cands, key=lambda m: (d.get(m, 10**6), not rm.is_turnout(m), m))
    # PIBT ordering: guide step first, then every option (including staying) by distance to goal.
    # Staying ranks by its own distance, so a blocked robot waits rather than backing away; ties
    # prefer staying (no wasted motion) and then turnouts.
    cands = [m for m in rm.neighbors(node) if m not in bl] + [node]
    cands.sort(key=lambda m: (m != nxt, d.get(m, 10**6), m != node, not rm.is_turnout(m), m))
    return cands

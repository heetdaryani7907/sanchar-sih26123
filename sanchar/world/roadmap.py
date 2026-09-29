"""Warehouse roadmap: a graph of nodes robots move between, one node per coordination tick.

Built from an ASCII layout:
    '#' wall   '.' free aisle   'T' turnout/lay-by   'S' station (pickup/drop)   'C' charger
Single-lane corridors are auto-detected as maximal chains of degree-2 free cells (turnouts break
a chain). Each corridor gets an id and two ordered ends so a corridor direction is well defined.

The coordinator only needs: neighbors(), dist_map(goal), corridor_of(), corridor_ends(),
is_turnout(). A continuous execution layer (planned) would interpolate between node positions
and never change topology.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from functools import lru_cache


@dataclass(frozen=True)
class Node:
    id: int
    x: float
    y: float
    kind: str                     # aisle | turnout | station | charger
    corridor: int | None = None   # single-lane corridor id, if any


@dataclass
class Roadmap:
    nodes: dict[int, Node]
    adj: dict[int, list[int]]
    corridors: dict[int, list[int]] = field(default_factory=dict)   # cid -> ordered node ids
    mouths: dict[int, tuple[int | None, int | None]] = field(default_factory=dict)
    cidx: dict[int, dict[int, int]] = field(default_factory=dict)
    width: int = 0
    height: int = 0

    # ---- topology -------------------------------------------------------------------------
    def neighbors(self, n: int) -> list[int]:
        return self.adj[n]

    def pos(self, n: int) -> tuple[float, float]:
        nd = self.nodes[n]
        return nd.x, nd.y

    def is_turnout(self, n: int) -> bool:
        return self.nodes[n].kind == "turnout"

    def corridor_of(self, n: int) -> int | None:
        return self.nodes[n].corridor

    def corridor_ends(self, cid: int) -> tuple[int | None, int | None]:
        """(mouth outside end A, mouth outside end B) of corridor cid."""
        return self.mouths[cid]

    def corridor_direction(self, cid: int, from_node: int, to_node: int) -> int:
        """+1 moving from end A towards end B, -1 from B towards A, 0 into or out of a pocket."""
        chain = self.corridors[cid]
        idx = self.cidx[cid]
        a, b = self.mouths[cid]
        fi, ti = idx.get(from_node), idx.get(to_node)
        if fi is not None and ti is not None:
            return 1 if ti > fi else -1
        if ti is not None:                                   # entering
            return 1 if from_node == a else (-1 if from_node == b else 0)
        if fi is not None:                                   # exiting
            if self.is_turnout(to_node):
                return 0
            if len(chain) == 1:
                return -1 if to_node == a else 1
            return 1 if fi == len(chain) - 1 else -1
        return 0

    def path_direction(self, cid: int, path: list[int]) -> int:
        """Direction a planned path takes through corridor cid (0 if it only waits or parks)."""
        for u, v in zip(path, path[1:]):
            if u != v and (self.corridor_of(u) == cid or self.corridor_of(v) == cid):
                d = self.corridor_direction(cid, u, v)
                if d != 0:
                    return d
        return 0

    def stations(self) -> list[int]:
        return [n for n, nd in self.nodes.items() if nd.kind == "station"]

    def chargers(self) -> list[int]:
        return [n for n, nd in self.nodes.items() if nd.kind == "charger"]

    # ---- distances -------------------------------------------------------------------------
    def dist_map(self, goal: int) -> dict[int, int]:
        return self._dist_map_cached(goal)

    @lru_cache(maxsize=None)
    def _dist_map_cached(self, goal: int) -> dict[int, int]:
        d = {goal: 0}
        q = deque([goal])
        while q:
            u = q.popleft()
            for v in self.adj[u]:
                if v not in d:
                    d[v] = d[u] + 1
                    q.append(v)
        return d

    def dist(self, a: int, b: int) -> int:
        return self.dist_map(b).get(a, 10**9)

    def __hash__(self):  # lets lru_cache key on identity
        return id(self)

    def __eq__(self, other):
        return self is other

    # ---- construction ----------------------------------------------------------------------
    @classmethod
    def from_ascii(cls, rows: list[str]) -> "Roadmap":
        rows = [r.rstrip("\n") for r in rows if r.strip()]
        h, w = len(rows), max(len(r) for r in rows)
        kinds = {".": "aisle", "T": "turnout", "S": "station", "C": "charger"}
        nodes: dict[int, Node] = {}
        grid: dict[tuple[int, int], int] = {}
        for y, r in enumerate(rows):
            for x, ch in enumerate(r.ljust(w, "#")):
                if ch in kinds:
                    nid = y * w + x
                    grid[(x, y)] = nid
                    nodes[nid] = Node(nid, float(x), float(y), kinds[ch])
        adj = {n: [] for n in nodes}
        for (x, y), n in grid.items():
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                m = grid.get((x + dx, y + dy))
                if m is not None:
                    adj[n].append(m)
        for n in adj:
            adj[n].sort()
        rm = cls(nodes, adj, {}, {}, {}, w, h)
        rm._detect_corridors()
        return rm

    def _detect_corridors(self) -> None:
        """Single-lane corridors = maximal stretches of 'lane' nodes: aisle nodes with at most two
        non-pocket neighbours. A lay-by pocket does NOT split a corridor, so a narrow passage with
        a pocket in the middle is one corridor (one traffic direction at a time; the pocket is a
        passing place for pushes and swaps)."""
        def lane(n: int) -> bool:
            return self.nodes[n].kind == "aisle" and sum(1 for m in self.adj[n] if not self.is_turnout(m)) <= 2
        lanes = {n for n in self.nodes if lane(n)}
        self.mouths, self.cidx = {}, {}
        seen: set[int] = set()
        for n0 in sorted(lanes):
            if n0 in seen:
                continue
            comp, stack = {n0}, [n0]
            while stack:
                u = stack.pop()
                for v in self.adj[u]:
                    if v in lanes and v not in comp:
                        comp.add(v)
                        stack.append(v)
            seen |= comp
            ends = [u for u in comp if sum(1 for v in self.adj[u] if v in comp) <= 1]
            start = min(ends) if ends else min(comp)
            chain, prev, cur = [start], None, start
            while True:
                nxt = [v for v in self.adj[cur] if v in comp and v != prev and v not in chain]
                if not nxt:
                    break
                prev, cur = cur, nxt[0]
                chain.append(cur)
            cid = len(self.corridors)
            self.corridors[cid] = chain
            self.cidx[cid] = {u: i for i, u in enumerate(chain)}
            for u in comp:
                nd = self.nodes[u]
                self.nodes[u] = Node(nd.id, nd.x, nd.y, nd.kind, cid)
            outside = lambda u: sorted(v for v in self.adj[u] if v not in comp and not self.is_turnout(v))
            oa, ob = outside(chain[0]), outside(chain[-1])
            ma = oa[0] if oa else None
            mb = (ob[-1] if len(chain) == 1 and len(ob) > 1 else (ob[0] if ob else None))
            self.mouths[cid] = (ma, mb)


# Reference layouts used by the benchmark.
# CORRIDOR: two open halls joined by a narrow single-lane passage with a lay-by pocket in the
# middle (the "narrow intersection / choke point" of the PS). WAREHOUSE: two-lane main aisles at
# top and bottom (stations on the outer lane), a single-lane middle cross aisle, and five narrow
# single-lane rack aisles: the realistic geometry of an AMR goods-to-person floor.
CORRIDOR_LAYOUT = [
    "###############",
    "#S..S#####S..S#",
    "#....##T##....#",
    "#S...........S#",
    "#....#####....#",
    "#S..S#####S..S#",
    "###############",
]

WAREHOUSE_LAYOUT = [
    "######################",
    "#S....S....S....S...S#",
    "#....................#",
    "#.####.####.####.###.#",
    "#.####.####.####.###.#",
    "#....................#",
    "#.####.####.####.###.#",
    "#.####.####.####.###.#",
    "#....................#",
    "#S....S....S....S..CS#",
    "######################",
]

# Single-lane 4-way intersection with one turnout per arm: the choke-point stress test.
CROSS_LAYOUT = [
    "###########",
    "#####S#####",
    "####T.#####",
    "#####.#T###",
    "#S.......S#",
    "###T#.#####",
    "#####.T####",
    "#####S#####",
    "###########",
]

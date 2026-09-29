"""Benchmark scenario families. Every family is built around OVERLAPPING paths, as the PS's
success criterion specifies. Each seed produces a different but reproducible instance; both
policies see the identical instance (same starts, same task sequences, same chaos)."""
from __future__ import annotations

import random

from sanchar.sim.engine import Chaos, Scenario, Task
from sanchar.world.roadmap import CORRIDOR_LAYOUT, CROSS_LAYOUT, WAREHOUSE_LAYOUT, Roadmap


def _xy(rm: Roadmap, n: int) -> tuple[int, int]:
    x, y = rm.pos(n)
    return int(x), int(y)


def corridor_swap(seed: int, n: int = 4, k: int = 4) -> Scenario:
    """Two halls joined by a narrow single-lane passage: every trip crosses it, half the robots
    in each direction."""
    rm = Roadmap.from_ascii(CORRIDOR_LAYOUT)
    rng = random.Random(seed)
    st = rm.stations()
    left = [s for s in st if _xy(rm, s)[0] < 7]
    right = [s for s in st if _xy(rm, s)[0] > 7]
    starts = rng.sample(left, n // 2) + rng.sample(right, n - n // 2)
    tasks = []
    for i, s in enumerate(starts):
        home, away = (left, right) if s in left else (right, left)
        tasks.append([Task(f"R{i}-{j}", rng.choice(away), rng.choice(home)) for j in range(k)])
    return Scenario("corridor_swap", CORRIDOR_LAYOUT, starts, tasks,
                    description=f"{n} AMRs crossing a narrow single-lane passage in both directions")


def intersection_cross(seed: int, n: int = 4, k: int = 3) -> Scenario:
    """Each robot repeatedly drives to the opposite arm of a single-lane 4-way intersection."""
    rm = Roadmap.from_ascii(CROSS_LAYOUT)
    rng = random.Random(seed)
    st = rm.stations()
    cx = sum(_xy(rm, s)[0] for s in st) / len(st)
    cy = sum(_xy(rm, s)[1] for s in st) / len(st)

    def opposite(s: int) -> int:
        x, y = _xy(rm, s)
        return min(st, key=lambda o: abs(_xy(rm, o)[0] - (2 * cx - x)) + abs(_xy(rm, o)[1] - (2 * cy - y)))

    starts = rng.sample(st, n)
    tasks = [[Task(f"R{i}-{j}", opposite(s), s) for j in range(k)] for i, s in enumerate(starts)]
    return Scenario("intersection_cross", CROSS_LAYOUT, starts, tasks, max_ticks=800,
                    description=f"{n} AMRs, dead-end arms around one single-lane 4-way intersection (gridlock stress test)")


def warehouse_overlap(seed: int, n: int = 6, k: int = 4, loss: float = 0.0,
                      name: str = "warehouse_overlap") -> Scenario:
    """Top-row pickups to bottom-row drops (and back) through shared vertical aisles."""
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    rng = random.Random(seed)
    st = rm.stations()
    top = [s for s in st if _xy(rm, s)[1] <= 4]
    bot = [s for s in st if _xy(rm, s)[1] > 4]
    pool = st if n <= len(st) else st + [m for m in rm.nodes
                                         if rm.corridor_of(m) is None and _xy(rm, m)[1] in (1, 2, 8, 9) and m not in st]
    starts = rng.sample(pool, n)
    tasks = []
    for i, s in enumerate(starts):
        seq = []
        for j in range(k):
            a, b = (top, bot) if (j + (s in bot)) % 2 == 0 else (bot, top)
            seq.append(Task(f"R{i}-{j}", rng.choice(a), rng.choice(b)))
        tasks.append(seq)
    desc = f"{n} AMRs, cross-warehouse trips through shared aisles"
    if loss:
        desc += f", {int(loss * 100)}% packet loss"
    return Scenario(name, WAREHOUSE_LAYOUT, starts, tasks, loss=loss, description=desc)


def blocked_aisle(seed: int, n: int = 6, k: int = 4) -> Scenario:
    """warehouse_overlap plus a pallet dropped in a busy vertical aisle from t=8 to t=90."""
    scn = warehouse_overlap(seed, n, k, name="blocked_aisle")
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    by_xy = {_xy(rm, nd): nd for nd in rm.nodes}
    x = random.Random(seed + 99).choice([6, 11, 16])
    blocked = [by_xy[(x, 6)]]
    scn.chaos = [Chaos(8, "block", blocked), Chaos(90, "unblock", blocked)]
    scn.description = f"{n} AMRs, one aisle blocked by a pallet for 82 s"
    return scn


FAMILIES = {
    "corridor_swap": corridor_swap,
    "intersection_cross": intersection_cross,
    "warehouse_overlap": warehouse_overlap,
    "blocked_aisle": blocked_aisle,
    "warehouse_loss30": lambda seed: warehouse_overlap(seed, loss=0.3, name="warehouse_loss30"),
    "warehouse_12": lambda seed: warehouse_overlap(seed, n=12, k=3, name="warehouse_12"),
}

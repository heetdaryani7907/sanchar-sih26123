"""Live-order scenario families (tasks arrive over time; robots allocate them themselves)."""
from __future__ import annotations

import random

from sanchar.sim.engine import Chaos
from sanchar.sim.market import MarketScenario, Order
from sanchar.world.roadmap import WAREHOUSE_LAYOUT, Roadmap


def _xy(rm, n):
    x, y = rm.pos(n)
    return int(x), int(y)


def orders_stream(seed: int, n_robots: int = 6, n_orders: int = 36, gap: float = 3.0,
                  name: str = "market_warehouse", loss: float = 0.0) -> MarketScenario:
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    rng = random.Random(seed)
    st = rm.stations()
    top = [s for s in st if _xy(rm, s)[1] <= 4]
    bot = [s for s in st if _xy(rm, s)[1] > 4]
    pool = st if n_robots <= len(st) else st + [m for m in rm.nodes if m not in st and rm.corridor_of(m) is None
                                                and rm.nodes[m].kind == "aisle" and _xy(rm, m)[1] in (2, 8)]
    starts = rng.sample(pool, n_robots)
    orders, t = [], 0.0
    for i in range(n_orders):
        t += rng.expovariate(1.0 / gap)
        a, b = (top, bot) if rng.random() < 0.5 else (bot, top)
        orders.append(Order(f"T{i:03d}", int(t), rng.choice(a), rng.choice(b)))
    return MarketScenario(name, WAREHOUSE_LAYOUT, starts, orders, loss=loss,
                          description=f"{n_robots} AMRs, {n_orders} live orders (Poisson, 1 per {gap:.0f} s)")


def market_blocked(seed: int) -> MarketScenario:
    scn = orders_stream(seed, name="market_blocked")
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    by_xy = {_xy(rm, n): n for n in rm.nodes}
    x = random.Random(seed + 5).choice([6, 11, 16])
    blocked = [by_xy[(x, 3)], by_xy[(x, 6)]]
    scn.chaos = [Chaos(15, "block", blocked), Chaos(120, "unblock", blocked)]
    scn.description += f"; aisle x={x} blocked from 15 s to 120 s"
    return scn


def market_crash(seed: int) -> MarketScenario:
    scn = orders_stream(seed, name="market_crash")
    victim = random.Random(seed + 11).randrange(len(scn.starts))
    scn.chaos = [Chaos(30, "crash", victim), Chaos(120, "remove", victim)]
    scn.description += f"; robot R{victim} dies at 30 s mid-task, technician removes it at 120 s"
    return scn


def market_deadzone(seed: int) -> MarketScenario:
    """A Wi-Fi dead zone (5 cells) appears over a busy junction from 20 s to 140 s."""
    scn = orders_stream(seed, name="market_deadzone")
    rm = Roadmap.from_ascii(WAREHOUSE_LAYOUT)
    by_xy = {_xy(rm, n): n for n in rm.nodes}
    cx = random.Random(seed + 23).choice([6, 11, 16])
    cy = random.Random(seed + 29).choice([2, 5, 8])
    centre = by_xy[(cx, cy)]
    zone = [centre, *rm.neighbors(centre)]
    scn.chaos = [Chaos(20, "deadzone", zone), Chaos(140, "clear_deadzone", zone)]
    scn.description += f"; Wi-Fi dead zone around ({cx},{cy}) from 20 s to 140 s"
    return scn


MARKET_FAMILIES = {
    "market_warehouse": orders_stream,
    "market_blocked": market_blocked,
    "market_crash": market_crash,
    "market_loss30": lambda seed: orders_stream(seed, name="market_loss30", loss=0.3),
    "market_deadzone": market_deadzone,
    "market_12": lambda seed: orders_stream(seed, n_robots=12, n_orders=60, gap=1.5, name="market_12"),
}

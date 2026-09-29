"""Physics checks cannot be fooled, and the full live-order loop works end to end."""
from bench.market_scenarios import market_blocked, market_crash, market_deadzone, orders_stream
from sanchar.sim.engine import World, apply_moves
from sanchar.sim.market import run_market, SancharMarketPolicy
from sanchar.world.roadmap import CORRIDOR_LAYOUT, Roadmap


def _world(starts):
    rm = Roadmap.from_ascii(CORRIDOR_LAYOUT)
    n = len(starts)
    return rm, World(rm, pos=dict(enumerate(starts)), alive=set(range(n)), present=set(range(n)))


def test_collision_swap_teleport_and_obstacle_are_counted():
    rm, w = _world([])
    a = rm.corridors[0][0]
    b = rm.corridors[0][1]
    c = rm.corridors[0][2]
    rm, w = _world([a, c])
    assert apply_moves(w, {0: b, 1: b})[0] == 1            # both enter b
    rm, w = _world([a, b])
    assert apply_moves(w, {0: b, 1: a})[1] == 1            # head-on swap through each other
    far = rm.corridors[0][-1]
    rm, w = _world([a])
    assert apply_moves(w, {0: far})[3] == 1 and w.pos[0] == a   # illegal jump refused
    rm, w = _world([a])
    w.obstacles = {b}
    assert apply_moves(w, {0: b})[2] == 1                   # drove into a pallet


def _ok(res):
    return res["finished"] and res["collisions"] + res["swaps"] + res["obstacle_hits"] == 0


def test_live_orders_complete_without_collisions():
    assert _ok(run_market(orders_stream(0, n_orders=20), SancharMarketPolicy(), 0))


def test_blocked_aisle_crash_and_dead_zone_are_survived():
    assert _ok(run_market(market_blocked(1), SancharMarketPolicy(), 1))
    assert _ok(run_market(market_crash(2), SancharMarketPolicy(), 2))
    assert _ok(run_market(market_deadzone(3), SancharMarketPolicy(), 3))

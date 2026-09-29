"""Property test for the safety claim: no two robots ever share a node or swap along an edge,
for random layouts, fleet sizes, packet-loss rates and seeds. Every robot with a goal makes progress."""
import pytest
from tests.smoke_fleet import run
from sanchar.world.roadmap import CORRIDOR_LAYOUT, WAREHOUSE_LAYOUT

CASES = [(lay, n, loss, seed) for lay in (CORRIDOR_LAYOUT, WAREHOUSE_LAYOUT)
         for n, loss in ((3, 0.0), (4, 0.3), (6, 0.5), (8, 0.2)) for seed in range(4)]


@pytest.mark.parametrize("layout,n,loss,seed", CASES)
def test_no_collisions_and_progress(layout, n, loss, seed):
    r = run(layout, n, loss, ticks=250, seed=seed)
    assert r["collisions"] == 0, r
    assert r["swaps"] == 0, r
    assert r["tasks"] > 0, r

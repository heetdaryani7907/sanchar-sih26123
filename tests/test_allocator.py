"""Task market CRDT: the merge rule must be commutative, associative and idempotent, so any
gossip order converges; concurrent claims resolve to the cheaper bid; re-opens win."""
import itertools
import random

from hypothesis import given, settings
from hypothesis import strategies as st

from sanchar.protocol.messages import TaskAnnounce
from sanchar.robot.allocator import merge

records = st.builds(
    TaskAnnounce,
    task_id=st.just("T1"), pickup=st.just(1), dropoff=st.just(2), created_tick=st.just(0), origin=st.just(-1),
    version=st.integers(0, 3), status=st.sampled_from(["open", "awarded", "picked", "done"]),
    owner=st.one_of(st.none(), st.integers(0, 5)), cost=st.floats(0, 50, allow_nan=False),
).map(lambda r: r if (r.status == "open") == (r.owner is None) else
      TaskAnnounce(task_id="T1", pickup=1, dropoff=2, created_tick=0, origin=-1, version=r.version,
                   status="awarded" if r.owner is not None else "open", owner=r.owner, cost=r.cost))


def key(r):
    return (r.version, r.status, r.owner, r.cost)


@settings(max_examples=300, deadline=None)
@given(records, records)
def test_merge_commutative(a, b):
    assert key(merge(a, b)) == key(merge(b, a))


@settings(max_examples=300, deadline=None)
@given(records, records, records)
def test_merge_associative(a, b, c):
    assert key(merge(merge(a, b), c)) == key(merge(a, merge(b, c)))


@settings(max_examples=200, deadline=None)
@given(records)
def test_merge_idempotent(a):
    assert key(merge(a, a)) == key(a)


def test_any_gossip_order_converges():
    rng = random.Random(4)
    recs = [TaskAnnounce(task_id="T1", pickup=1, dropoff=2, created_tick=0, origin=-1, version=0,
                         status="awarded", owner=o, cost=c) for o, c in ((1, 7.0), (2, 5.0), (3, 5.0))]
    results = set()
    for perm in itertools.permutations(recs):
        cur = None
        for r in perm:
            cur = merge(cur, r)
        results.add(key(cur))
    assert results == {(0, "awarded", 2, 5.0)}        # cheapest bid, ties to lower robot id
    reopened = TaskAnnounce(task_id="T1", pickup=1, dropoff=2, created_tick=0, origin=-1, version=1)
    assert merge(recs[1], reopened).status == "open"  # a re-open (higher version) beats stale claims

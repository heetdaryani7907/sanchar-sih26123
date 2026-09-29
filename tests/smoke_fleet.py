"""In-process fleet harness: N coordinators, a lossy broadcast bus, continuous task stream.
Checks the two hard success-criteria ingredients: zero collisions (node + swap) and progress.
sim/runner.py runs the same loop over real UDP with one OS process per robot."""
import random, sys
from sanchar.world.roadmap import Roadmap, CORRIDOR_LAYOUT, WAREHOUSE_LAYOUT
from sanchar.robot.coordinator import Coordinator, Config


def run(layout, n, loss=0.0, ticks=300, seed=0, verbose=False, trace=None, lossless_first_tick=False):
    rm = Roadmap.from_ascii(layout)
    rng = random.Random(seed)
    stations = rm.stations()
    free = [nd for nd in rm.nodes if not rm.is_turnout(nd) and rm.corridor_of(nd) is None]
    starts = rng.sample(stations if len(stations) >= n else (free if len(free) >= n else list(rm.nodes)), n)
    coords = [Coordinator(rm, i, Config(seed=seed), roster=set(range(n))) for i in range(n)]
    done, collisions, swaps, wait_total = [0] * n, 0, 0, 0
    def new_goal(i):
        c = coords[i]
        opts = [s for s in stations if s != c.node]
        c.set_goal(rng.choice(opts), task_id=f"T{i}-{done[i]}", load=bool(done[i] % 2))
    for i, c in enumerate(coords):
        c.set_pose(starts[i]); new_goal(i)
    for t in range(ticks):
        claims = [c.claim_phase(t) for c in coords]
        for i, (hello, intent, _, _) in enumerate(claims):
            for j, c in enumerate(coords):
                if i != j and (rng.random() >= loss or (lossless_first_tick and t == 0)):
                    c.ingest(hello, 2 * t); c.ingest(intent, 2 * t)
        prev = [c.node for c in coords]
        ds = [c.move_phase(t) for c in coords]
        if trace is not None: trace.append((t, [(d.node, d.claim, d.reason, list(d.events)) for d in ds]))
        occ = {}
        for i, d in enumerate(ds):
            if d.node in occ:
                collisions += 1
                if verbose: print(f"t={t} COLLISION R{i} & R{occ[d.node]} at {d.node}")
            occ[d.node] = i
        for i in range(n):
            for j in range(i + 1, n):
                if ds[i].moved and ds[j].moved and prev[i] == ds[j].node and prev[j] == ds[i].node:
                    swaps += 1
        for i, c in enumerate(coords):
            if c.node == c.goal:
                done[i] += 1; new_goal(i)
            if not ds[i].moved and c.goal != c.node: wait_total += 1
        for i, d in enumerate(ds):
            for j, c in enumerate(coords):
                if i != j and (rng.random() >= loss or (lossless_first_tick and t == 0)):
                    c.ingest(d.hello, 2 * t + 1)
    m = {k: sum(c.metrics[k] for c in coords) for k in coords[0].metrics}
    return dict(coords=coords, collisions=collisions, swaps=swaps, tasks=sum(done), per_robot=done, wait_ticks=wait_total, **m)


if __name__ == "__main__":
    cases = [("corridor", CORRIDOR_LAYOUT, 3, 0.0), ("corridor", CORRIDOR_LAYOUT, 4, 0.3),
             ("warehouse", WAREHOUSE_LAYOUT, 6, 0.0), ("warehouse", WAREHOUSE_LAYOUT, 8, 0.4),
             ("warehouse", WAREHOUSE_LAYOUT, 12, 0.2)]
    bad = 0
    for name, lay, n, loss in cases:
        for seed in range(3):
            r = run(lay, n, loss, ticks=300, seed=seed)
            flag = "" if r["collisions"] == 0 and r["swaps"] == 0 and min(r["per_robot"]) > 0 else "  <-- PROBLEM"
            bad += bool(flag)
            r.pop('coords')
            print(f"{name:9s} n={n:2d} loss={loss:.1f} seed={seed}: collisions={r['collisions']} swaps={r['swaps']} tasks={r['tasks']:3d} per_robot={r['per_robot']} wait_ticks={r["wait_ticks"]} cycles={r['cycles_broken']}{flag}")
    sys.exit(1 if bad else 0)

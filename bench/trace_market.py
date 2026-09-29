"""Debug: python -m bench.trace_market <family> <seed> <t_from> <n>"""
import sys
from bench.market_scenarios import MARKET_FAMILIES
from sanchar.sim.market import run_market, SancharMarketPolicy
from sanchar.world.roadmap import Roadmap
fam, seed, t0, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
scn = MARKET_FAMILIES[fam](seed); rm = Roadmap.from_ascii(scn.layout)
P = lambda x: None if x is None else (int(rm.pos(x)[0]), int(rm.pos(x)[1]))
pol = SancharMarketPolicy()
def on_tick(w, info):
    if t0 <= info["t"] < t0 + n:
        parts = []
        for r in sorted(w.present):
            a = pol.agents[r]; d = pol.last_decisions.get(r); m = pol.markets[r]
            parts.append(f"R{r}{'' if r in w.alive else 'X'}{P(w.pos[r])}g{P(a.goal)} {m.my_task} {d.reason[:22] if d else ''}")
        print(info["t"], info["done"], " | ".join(parts))
res = run_market(scn, pol, seed, on_tick=on_tick)
print({k: res[k] for k in ("makespan", "tasks_done", "tasks_total", "collisions")})
# union view of task records
recs = {}
from sanchar.robot.allocator import merge
for m in pol.markets.values():
    for tid, rec in m.tasks.items(): recs[tid] = merge(recs.get(tid), rec)
print("not done:", [(tid, r.status, r.owner, r.version, P(r.pickup), P(r.dropoff)) for tid, r in sorted(recs.items()) if r.status != "done"])
print("per-robot views of not-done:", {r: [(tid, m.tasks[tid].status, m.tasks[tid].owner, m.tasks[tid].version) for tid in recs if tid in m.tasks and recs[tid].status != "done"] for r, m in pol.markets.items()})

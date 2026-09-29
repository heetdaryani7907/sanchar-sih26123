"""Debug: python -m bench.trace_base <family> <seed> <t_from> <n>"""
import sys
from bench.scenarios import FAMILIES
from baseline.stop_and_wait import StopAndWaitPolicy
from sanchar.sim.engine import run
from sanchar.world.roadmap import Roadmap
fam, seed, t0, n = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
scn = FAMILIES[fam](seed); rm = Roadmap.from_ascii(scn.layout)
P = lambda x: None if x is None else (int(rm.pos(x)[0]), int(rm.pos(x)[1]))
pol = StopAndWaitPolicy()
def on_tick(w, info):
    if t0 <= info["t"] < t0 + n:
        print(info["t"], info["completion"], " ".join(f"R{r}{P(w.pos[r])}>{P(pol.goal[r])} nx{P(pol.path[r][1]) if len(pol.path[r])>1 else None} w{pol.wait[r]}h{pol.hold[r]}" for r in sorted(w.present)))
r = run(scn, pol, seed, on_tick=on_tick)
print({k: r[k] for k in ("makespan", "tasks_done", "tasks_total", "backoffs")})

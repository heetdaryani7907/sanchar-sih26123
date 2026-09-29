"""Debug helper: python -m bench.trace <family> <seed> [sanchar|baseline] [from_tick] [n_ticks]"""
import sys
from bench.scenarios import FAMILIES
from baseline.stop_and_wait import StopAndWaitPolicy
from sanchar.sim.engine import run
from sanchar.sim.policies import SancharPolicy
from sanchar.world.roadmap import Roadmap

fam, seed = sys.argv[1], int(sys.argv[2])
polname = sys.argv[3] if len(sys.argv) > 3 else "sanchar"
t_from = int(sys.argv[4]) if len(sys.argv) > 4 else 0
n_t = int(sys.argv[5]) if len(sys.argv) > 5 else 30
scn = FAMILIES[fam](seed)
rm = Roadmap.from_ascii(scn.layout)
P = lambda n: None if n is None else (int(rm.pos(n)[0]), int(rm.pos(n)[1]))
pol = SancharPolicy() if polname == "sanchar" else StopAndWaitPolicy()
def on_tick(w, info):
    t = info["t"]
    if t_from <= t < t_from + n_t:
        parts = []
        for r in sorted(w.present):
            if polname == "sanchar" and r in pol.last_decisions:
                d = pol.last_decisions[r]
                a = pol.agents[r]
                parts.append(f"R{r}{P(w.pos[r])}g{P(a.goal)}c{P(d.claim)} {d.reason[:26]}")
            else:
                parts.append(f"R{r}{P(w.pos[r])}")
        print(t, info["completion"], " | ".join(parts))
res = run(scn, pol, seed, on_tick=on_tick)
print({k: res[k] for k in ("makespan", "finished", "tasks_done", "tasks_total", "collisions")})

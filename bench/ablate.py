"""Quick ablation: python -m bench.ablate fam1,fam2 seeds  (one-factor-at-a-time over Config)"""
import json, statistics, sys
from bench.scenarios import FAMILIES
from sanchar.robot.coordinator import Config
from sanchar.sim.engine import run
from sanchar.sim.policies import SancharPolicy
fams = sys.argv[1].split(","); seeds = int(sys.argv[2])
variants = [json.loads(v) for v in sys.argv[3:]] if len(sys.argv) > 3 else [{}]
for kw in variants:
    out = []
    for fam in fams:
        rs = [run(FAMILIES[fam](s), SancharPolicy(Config(**kw)), s) for s in range(seeds)]
        out.append(f"{fam}:{statistics.fmean(r['makespan'] for r in rs):.0f}/{sum(not r['finished'] for r in rs)}u/{sum(r['collisions'] for r in rs)}c")
    print(kw or "default", " ".join(out), flush=True)

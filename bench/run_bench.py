"""Benchmark: SANCHAR vs stop-and-wait on every scenario family, many seeds, paired statistics.

Usage:  python -m bench.run_bench [--seeds 30] [--families a,b] [--out bench/results]
Writes bench/results/<family>.json (per-seed rows) and bench/results/summary.json + summary.md.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

from baseline.stop_and_wait import StopAndWaitPolicy
from bench.market_scenarios import MARKET_FAMILIES
from bench.scenarios import FAMILIES
from sanchar.sim.engine import run
from sanchar.sim.market import CentralDispatchBaseline, SancharMarketPolicy, run_market
from sanchar.sim.policies import SancharPolicy

T95 = {5: 2.776, 10: 2.262, 20: 2.093, 30: 2.045, 50: 2.010}


def ci95(xs: list[float]) -> tuple[float, float]:
    n = len(xs)
    m = statistics.fmean(xs)
    if n < 2:
        return m, 0.0
    tcrit = T95.get(n, 1.96 if n > 50 else 2.1)
    return m, tcrit * statistics.stdev(xs) / math.sqrt(n)


def bench_family(name: str, seeds: int) -> dict:
    rows = []
    market = name in MARKET_FAMILIES
    gen = MARKET_FAMILIES[name] if market else FAMILIES[name]
    for seed in range(seeds):
        if market:
            rb = run_market(gen(seed), CentralDispatchBaseline(), seed)
            rs = run_market(gen(seed), SancharMarketPolicy(), seed)
        else:
            rb = run(gen(seed), StopAndWaitPolicy(), seed)
            rs = run(gen(seed), SancharPolicy(), seed)
        rows.append({"seed": seed, "baseline": rb, "sanchar": rs})

    def pct(key):
        return [100 * (r["baseline"][key] - r["sanchar"][key]) / r["baseline"][key] for r in rows]

    def agg(key, pol):
        return ci95([float(r[pol][key]) for r in rows])

    def bad(pol):
        return sum(r[pol]["collisions"] + r[pol]["swaps"] + r[pol]["obstacle_hits"] for r in rows)

    imp = pct("makespan")
    fin = [r for r in rows if r["baseline"]["finished"] and r["sanchar"]["finished"]]
    imp_fin = [100 * (r["baseline"]["makespan"] - r["sanchar"]["makespan"]) / r["baseline"]["makespan"] for r in fin]
    s = {
        "family": name, "seeds": seeds, "robots": rows[0]["sanchar"]["robots"],
        "description": gen(0).description, "mode": "live orders" if market else "fixed tasks",
        "baseline_makespan": agg("makespan", "baseline"), "sanchar_makespan": agg("makespan", "sanchar"),
        "baseline_sum": agg("mean_latency" if market else "sum_completion", "baseline"),
        "sanchar_sum": agg("mean_latency" if market else "sum_completion", "sanchar"),
        "makespan_reduction_pct": ci95(imp), "sum_completion_reduction_pct": ci95(pct("mean_latency" if market else "sum_completion")),
        "sanchar_collisions": bad("sanchar"), "baseline_collisions": bad("baseline"),
        "sanchar_unfinished": sum(not r["sanchar"]["finished"] for r in rows),
        "baseline_unfinished": sum(not r["baseline"]["finished"] for r in rows),
        "seeds_sanchar_better": sum(i > 0 for i in imp),
        "baseline_makespan_median": statistics.median(r["baseline"]["makespan"] for r in rows),
        "sanchar_makespan_median": statistics.median(r["sanchar"]["makespan"] for r in rows),
        "both_finished_seeds": len(fin),
        "makespan_reduction_pct_both_finished": ci95(imp_fin) if len(imp_fin) >= 2 else (float("nan"), float("nan")),
        "baseline_gridlocked_seeds": sum(not r["baseline"]["finished"] for r in rows),
        "bytes_per_robot_tick": statistics.fmean(r["sanchar"]["bytes_per_robot_tick"] for r in rows),
        "decision_ms_p50": statistics.fmean(r["sanchar"]["decision_ms_p50"] for r in rows),
        "decision_ms_p99": max(r["sanchar"]["decision_ms_p99"] for r in rows),
        "baseline_backoffs": sum(r["baseline"]["backoffs"] for r in rows),
    }
    return {"summary": s, "rows": rows}


def fmt(ci) -> str:
    return f"{ci[0]:.1f} ± {ci[1]:.1f}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--families", default=",".join(list(FAMILIES) + list(MARKET_FAMILIES)))
    ap.add_argument("--out", default="bench/results")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    summaries = []
    for fam in a.families.split(","):
        t0 = time.time()
        res = bench_family(fam, a.seeds)
        (out / f"{fam}.json").write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
        s = res["summary"]
        summaries.append(s)
        print(f"{fam:20s} n={s['robots']:2d} base {fmt(s['baseline_makespan'])} sanchar {fmt(s['sanchar_makespan'])}"
              f" reduction {fmt(s['makespan_reduction_pct'])}% coll S/B {s['sanchar_collisions']}/{s['baseline_collisions']}"
              f" unfinished S/B {s['sanchar_unfinished']}/{s['baseline_unfinished']} better {s['seeds_sanchar_better']}/{a.seeds}"
              f" backoffs {s['baseline_backoffs']} [{time.time() - t0:.0f}s]", flush=True)
    (out / "summary.json").write_text(json.dumps(summaries, indent=1), encoding="utf-8")
    lines = ["| Scenario | Mode | Robots | Stop-and-wait makespan, median (s) | SANCHAR makespan, median (s) | "
             "Makespan reduction, all seeds | Makespan reduction, seeds where both finished | "
             "Latency or sum-of-completion reduction | Baseline gridlocked | SANCHAR unfinished | "
             "Collisions S / B | SANCHAR better |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for s in summaries:
        both = (f"{fmt(s['makespan_reduction_pct_both_finished'])}% (n={s['both_finished_seeds']})"
                if s["both_finished_seeds"] >= 2 else "n/a (baseline never finished)")
        lines.append(f"| {s['family']} | {s['mode']} | {s['robots']} | {s['baseline_makespan_median']:.0f} | "
                     f"{s['sanchar_makespan_median']:.0f} | {fmt(s['makespan_reduction_pct'])}% | {both} | "
                     f"{fmt(s['sum_completion_reduction_pct'])}% | {s['baseline_gridlocked_seeds']}/{s['seeds']} | "
                     f"{s['sanchar_unfinished']}/{s['seeds']} | {s['sanchar_collisions']} / {s['baseline_collisions']} | "
                     f"{s['seeds_sanchar_better']}/{s['seeds']} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

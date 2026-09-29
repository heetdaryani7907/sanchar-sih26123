"""Train and evaluate the on-robot ETA model from simulation logs.

    python -m sanchar.edge_ai.train [--train-seeds 40] [--test-seeds 20]

1. Runs live-order simulations with the plain (path-length) bidder and logs every leg: the
   features a robot knew when the leg started, and how long the leg really took.
2. Fits a ridge regression on training seeds; reports mean absolute error on held-out seeds
   against the plain path-length predictor.
3. A/B test: re-runs held-out scenarios with robots bidding on the learned ETA and reports the
   change in completion time and order latency.
Writes sanchar/edge_ai/eta_model.json and bench/results/edge_ai.json.
"""
from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

import numpy as np

from bench.market_scenarios import MARKET_FAMILIES
from sanchar.edge_ai.eta_model import DEFAULT_PATH, FEATURES, EtaModel, attach, leg_features
from sanchar.sim.market import SancharMarketPolicy, run_market

ROOT = Path(__file__).resolve().parents[2]


class LoggingPolicy(SancharMarketPolicy):
    """SANCHAR policy that records (features at leg start, actual leg duration) for every robot."""

    def __init__(self, model: EtaModel | None = None, **kw):
        super().__init__(**kw)
        self.model = model

    def reset(self, world, seed):
        super().reset(world, seed)
        self.legs: dict[int, tuple[int, int, np.ndarray]] = {}
        self.samples: list[tuple[np.ndarray, float]] = []
        if self.model is not None:
            for rid, m in self.markets.items():
                attach(m, self.agents[rid], self.model)

    def _pre_claim(self, rid, world):
        a = self.agents[rid]
        before = a.goal
        out = super()._pre_claim(rid, world)
        t = world.tick
        leg = self.legs.get(rid)
        if leg is not None and (a.node == leg[1] or a.goal != leg[1]):
            if a.node == leg[1]:
                self.samples.append((leg[2], float(t - leg[0])))
            del self.legs[rid]
        if a.goal is not None and a.goal != before and a.goal != a.node and rid not in self.legs:
            peers = [(pv.hello.node, pv.intent or []) for pv in a.peers.values()]
            x = leg_features(world.rm, a.node, a.goal, set(a.obstacles), peers)
            if x is not None:
                self.legs[rid] = (t, a.goal, x)
        return out


def collect(seeds, families=("market_warehouse", "market_blocked", "market_loss30", "market_12")):
    X, y = [], []
    for fam in families:
        for seed in seeds:
            pol = LoggingPolicy()
            run_market(MARKET_FAMILIES[fam](seed), pol, seed)
            for x, dur in pol.samples:
                X.append(x)
                y.append(dur)
    return np.array(X), np.array(y)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--train-seeds", type=int, default=40)
    ap.add_argument("--test-seeds", type=int, default=20)
    a = ap.parse_args(argv)
    t0 = time.time()
    train = range(1000, 1000 + a.train_seeds)
    test = range(2000, 2000 + a.test_seeds)
    Xtr, ytr = collect(train)
    Xte, yte = collect(test)
    model = EtaModel().fit(Xtr, ytr)
    pred = np.array([model.predict(x) for x in Xte])
    mae_model = float(np.mean(np.abs(pred - yte)))
    mae_len = float(np.mean(np.abs(Xte[:, 0] - yte)))
    print(f"legs: train {len(ytr)}  test {len(yte)}")
    print(f"ETA error on held-out seeds: path length {mae_len:.2f} s  ->  learned model {mae_model:.2f} s "
          f"({100 * (mae_len - mae_model) / mae_len:.0f}% lower)")
    print("weights:", dict(zip(FEATURES, model.w.round(3))))

    ab = {}
    for fam in ("market_warehouse", "market_12", "market_blocked", "market_loss30", "market_crash"):
        plain = [run_market(MARKET_FAMILIES[fam](s), SancharMarketPolicy(), s) for s in test]
        smart = [run_market(MARKET_FAMILIES[fam](s), LoggingPolicy(model=model), s) for s in test]
        mk = [100 * (p["makespan"] - q["makespan"]) / p["makespan"] for p, q in zip(plain, smart)]
        lat = [100 * (p["mean_latency"] - q["mean_latency"]) / p["mean_latency"] for p, q in zip(plain, smart)]
        coll = sum(q["collisions"] + q["swaps"] + q["obstacle_hits"] for q in smart)
        ab[fam] = {"makespan_change_pct": statistics.fmean(mk), "latency_change_pct": statistics.fmean(lat),
                   "seeds_better_makespan": sum(v > 0 for v in mk), "seeds": len(mk), "collisions": coll,
                   "unfinished": sum(not q["finished"] for q in smart)}
        print(f"A/B {fam:17s}: makespan {ab[fam]['makespan_change_pct']:+.1f}% better, latency "
              f"{ab[fam]['latency_change_pct']:+.1f}% better, {ab[fam]['seeds_better_makespan']}/{len(mk)} seeds, "
              f"collisions {coll}, unfinished {ab[fam]['unfinished']}")
    meta = {"train_legs": len(ytr), "test_legs": len(yte), "mae_path_length": mae_len, "mae_model": mae_model}
    model.save(DEFAULT_PATH, meta)
    (ROOT / "bench/results/edge_ai.json").write_text(json.dumps({**meta, "ab": ab,
                                                                 "weights": dict(zip(FEATURES, model.w.tolist()))}, indent=1))
    print(f"saved {DEFAULT_PATH} [{time.time() - t0:.0f}s]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

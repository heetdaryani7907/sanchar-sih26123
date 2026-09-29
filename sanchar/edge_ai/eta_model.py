"""Edge-AI: congestion-aware travel-time (ETA) prediction that runs on the robot.

Each robot predicts how long a leg will really take (not just its length) from what it already
knows through the radio: peers' positions and broadcast plans, single-lane corridors on the route,
and crowding near the start and the goal. The prediction replaces plain path length in auction
bids, so orders go to the robot that will actually get there first, not the one that is merely
closest.

The model is a ridge regression on engineered features: a few hundred bytes of weights and one
dot product per prediction (timing on a Raspberry Pi is not measured yet). It is trained offline
from simulation logs (`python -m sanchar.edge_ai.train`) and shipped as JSON.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from sanchar.robot.planner import static_path
from sanchar.world.roadmap import Roadmap

FEATURES = ["path_len", "corridor_nodes", "peers_near_path", "peers_near_start", "peers_near_goal",
            "plan_overlap", "opposing_in_corridors", "bias"]
DEFAULT_PATH = Path(__file__).with_name("eta_model.json")


def leg_features(rm: Roadmap, start: int, goal: int, obstacles: set[int], peers: list[tuple[int, list[int]]]) -> np.ndarray:
    """peers: [(node, intent_nodes)] as known from the radio."""
    path = static_path(rm, start, goal, obstacles)
    if path[-1] != goal:
        return None
    pset = set(path)
    near_path = set(pset)
    for n in path:
        near_path.update(rm.neighbors(n))
    corridors = {rm.corridor_of(n) for n in path if rm.corridor_of(n) is not None}
    d_start = rm.dist_map(start)
    d_goal = rm.dist_map(goal)
    peers_near_path = sum(1 for node, _ in peers if node in near_path)
    near_start = sum(1 for node, _ in peers if d_start.get(node, 99) <= 3)
    near_goal = sum(1 for node, _ in peers if d_goal.get(node, 99) <= 3)
    overlap = sum(len(pset.intersection(intent[1:])) for _, intent in peers)
    opposing = sum(1 for node, _ in peers if rm.corridor_of(node) in corridors)
    corr_nodes = sum(1 for n in path if rm.corridor_of(n) is not None)
    return np.array([len(path) - 1, corr_nodes, peers_near_path, near_start, near_goal, overlap, opposing, 1.0])


class EtaModel:
    def __init__(self, weights: np.ndarray | None = None):
        self.w = weights if weights is not None else np.array([1.0] + [0.0] * (len(FEATURES) - 1))

    def fit(self, X: np.ndarray, y: np.ndarray, l2: float = 1.0) -> "EtaModel":
        reg = l2 * np.eye(X.shape[1])
        reg[-1, -1] = 0.0                                   # do not shrink the bias
        self.w = np.linalg.solve(X.T @ X + reg, X.T @ y)
        return self

    def predict(self, x: np.ndarray) -> float:
        return float(max(x[0], x @ self.w))                # never faster than the path length

    def save(self, path: Path = DEFAULT_PATH, meta: dict | None = None) -> None:
        path.write_text(json.dumps({"features": FEATURES, "weights": self.w.round(5).tolist(), **(meta or {})}, indent=1))

    @classmethod
    def load(cls, path: Path = DEFAULT_PATH) -> "EtaModel":
        d = json.loads(Path(path).read_text())
        return cls(np.array(d["weights"]))


def attach(market, coord, model: EtaModel) -> None:
    """Make a robot's TaskMarket bid with predicted ETA instead of path length."""
    rm = market.rm

    def cost(node: int, target: int, obstacles: set[int]) -> float:
        if target in obstacles:
            return float("inf")
        peers = [(pv.hello.node, pv.intent or []) for pv in coord.peers.values()]
        x = leg_features(rm, node, target, obstacles, peers)
        return float("inf") if x is None else model.predict(x)

    market.cost_fn = cost

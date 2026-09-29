"""SANCHAR live fleet dashboard (zero dependencies: stdlib HTTP + Server-Sent Events).

    python -m sanchar.dashboard.server [--port 8080] [--robots 6] [--layout warehouse] [--speed 4]

Runs a live warehouse with a continuous order stream and serves a browser dashboard. The fleet
view is built ONLY from what a passive radio listener hears (robots' own signed Hello / Intent
broadcasts, with the same packet loss the robots suffer). The dashboard never plans, assigns or
arbitrates; killing it changes nothing for the fleet. Chaos controls inject physical events
(blocked aisle, dead zone, robot crash, packet loss, network partition) into the world model.

Switch the policy to "stop_and_wait" to watch the conventional centralized baseline on the same
floor with the same order stream.
"""
from __future__ import annotations

import argparse
import json
import queue
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from sanchar.protocol.messages import Hello, Intent
from sanchar.sim.engine import World, apply_moves
from sanchar.sim.market import CentralDispatchBaseline, Order, SancharMarketPolicy
from sanchar.world.roadmap import CORRIDOR_LAYOUT, CROSS_LAYOUT, WAREHOUSE_LAYOUT, Roadmap

LAYOUTS = {"warehouse": WAREHOUSE_LAYOUT, "corridor": CORRIDOR_LAYOUT, "cross": CROSS_LAYOUT}
WEB = Path(__file__).parent / "web"


class LiveSim:
    def __init__(self, layout: str = "warehouse", robots: int = 6, seed: int = 1,
                 policy: str = "sanchar", speed: float = 4.0, order_gap: float = 3.0):
        self.lock = threading.RLock()
        self.subs: list[queue.Queue] = []
        self.speed, self.paused = speed, False
        self.reset(layout, robots, seed, policy, order_gap)

    # ---- lifecycle ------------------------------------------------------------------------
    def reset(self, layout=None, robots=None, seed=None, policy=None, order_gap=None) -> None:
        with self.lock:
            self.layout_name = layout or getattr(self, "layout_name", "warehouse")
            self.n = robots or getattr(self, "n", 6)
            self.seed = seed if seed is not None else getattr(self, "seed", 1)
            self.policy_name = policy or getattr(self, "policy_name", "sanchar")
            self.order_gap = order_gap or getattr(self, "order_gap", 3.0)
            self.rm = Roadmap.from_ascii(LAYOUTS[self.layout_name])
            self.rng = random.Random(self.seed)
            st = self.rm.stations()
            pool = st if len(st) >= self.n else st + [m for m in self.rm.nodes if m not in st and self.rm.corridor_of(m) is None]
            starts = self.rng.sample(pool, self.n)
            self.world = World(self.rm, pos=dict(enumerate(starts)), alive=set(range(self.n)),
                               present=set(range(self.n)))
            self.world.battery = {r: self.rng.uniform(0.45, 1.0) for r in range(self.n)}
            self.policy = SancharMarketPolicy() if self.policy_name == "sanchar" else CentralDispatchBaseline()
            self.heard: dict[int, dict] = {}
            self.tap_bytes = 0
            self.tap_packets = 0
            self.tap_rng = random.Random(self.seed + 99)
            self.policy.tap = self._tap
            self.policy.reset(self.world, self.seed)
            self.policy.tap = self._tap
            self.t = 0
            self.next_order = 0.0
            self.order_seq = 0
            self.release: dict[str, int] = {}
            self.done_at: dict[str, int] = {}
            self.collisions = 0
            self.events: list[list] = []
            self.event_cursor = 0
            self.msg_window: list[tuple[int, int, int]] = []
            self.pending_chaos: list[tuple[str, object]] = []

    def _tap(self, src, src_node, body, rnd, nbytes) -> None:
        """Passive listener: hears broadcasts subject to the same loss and dead zones."""
        self.tap_packets += 1
        self.tap_bytes += nbytes
        if src_node in self.world.dead_zone or self.tap_rng.random() < self.world.loss:
            return
        h = self.heard.setdefault(src, {})
        if isinstance(body, Hello):
            h.update(hello=body, heard_t=self.t)
        elif isinstance(body, Intent):
            h.update(intent=body.nodes)

    # ---- control --------------------------------------------------------------------------
    def chaos(self, kind: str, arg=None) -> None:
        with self.lock:
            self.pending_chaos.append((kind, arg))

    def _apply_chaos(self) -> None:
        w = self.world
        for kind, arg in self.pending_chaos:
            if kind == "toggle_block":
                n = int(arg)
                w.obstacles ^= {n}
                self._event(-1, f"operator {'blocked' if n in w.obstacles else 'cleared'} node {self._xy(n)}")
            elif kind == "toggle_deadzone":
                n = int(arg)
                ball = {n} | set(self.rm.neighbors(n))
                if n in w.dead_zone:
                    w.dead_zone -= ball
                else:
                    w.dead_zone |= ball
                self._event(-1, f"Wi-Fi dead zone {'on' if n in w.dead_zone else 'off'} around {self._xy(n)}")
            elif kind == "crash":
                alive = sorted(w.alive)
                if alive:
                    victim = int(arg) if arg is not None and int(arg) in w.alive else self.rng.choice(alive)
                    w.alive.discard(victim)
                    self._event(victim, f"R{victim} powered off (crash) at {self._xy(w.pos[victim])}")
            elif kind == "loss":
                w.loss = max(0.0, min(0.9, float(arg)))
                self._event(-1, f"packet loss set to {int(w.loss * 100)}%")
            elif kind == "partition":
                half = sorted(w.alive)[: len(w.alive) // 2]
                w.partition = {r: (1 if r in half else 0) for r in w.alive}
                self._event(-1, f"network split: {half} cannot hear the rest")
            elif kind == "heal":
                w.partition = {}
                self._event(-1, "network healed")
            elif kind == "burst":
                for _ in range(int(arg or 6)):
                    self._new_order()
                self._event(-1, f"order burst: {int(arg or 6)} orders")
            elif kind == "clear":
                w.obstacles.clear()
                w.dead_zone.clear()
                w.partition = {}
                w.loss = 0.0
                self._event(-1, "all chaos cleared")
        self.pending_chaos = []

    def _xy(self, n: int) -> tuple[int, int]:
        x, y = self.rm.pos(n)
        return int(x), int(y)

    def _event(self, rid: int, text: str) -> None:
        self.events.append([self.t, rid, text])

    def _new_order(self) -> None:
        st = self.rm.stations()
        mid = (min(self.rm.pos(s)[1] for s in st) + max(self.rm.pos(s)[1] for s in st)) / 2
        top = [s for s in st if self.rm.pos(s)[1] <= mid] or st
        bot = [s for s in st if self.rm.pos(s)[1] > mid] or st
        a, b = (top, bot) if self.rng.random() < 0.5 else (bot, top)
        o = Order(f"T{self.order_seq:04d}", self.t, self.rng.choice(a), self.rng.choice(b))
        self.order_seq += 1
        self.release[o.task_id] = self.t
        self.policy.inject(o, self.world)

    # ---- simulation -----------------------------------------------------------------------
    def step(self) -> None:
        with self.lock:
            w = self.world
            w.tick = self.t
            self._apply_chaos()
            while self.next_order <= self.t:
                self._new_order()
                self.next_order += self.rng.expovariate(1.0 / self.order_gap)
            prev = dict(w.pos)
            pk0 = self.tap_packets
            moves = self.policy.step(w)
            c, sw, oh, tp = apply_moves(w, moves)
            self.collisions += c + sw + oh
            chargers = set(self.rm.chargers())
            for r in w.alive:
                moved = w.pos[r] != prev[r]
                b = w.battery[r] - 0.0002 - (0.0015 if moved else 0.0)
                if not moved and w.pos[r] in chargers:
                    b += 0.02
                w.battery[r] = max(0.0, min(1.0, b))
            for tid in self.policy.done_tasks():
                if tid not in self.done_at:
                    self.done_at[tid] = self.t + 1
            for t, rid, e in getattr(self.policy, "events", [])[-64:]:
                if t == self.t:
                    self._event(rid, e)
            for t, rid, e in getattr(self.policy, "market_events", [])[-64:]:
                if t == self.t:
                    self._event(rid, e)
            self.msg_window.append((self.t, self.tap_packets - pk0, 0))
            self.msg_window = self.msg_window[-20:]
            self.t += 1
            state = self.state()
        for q in list(self.subs):
            try:
                q.put_nowait(state)
            except queue.Full:
                pass

    def loop(self) -> None:
        while True:
            t0 = time.time()
            if not self.paused:
                try:
                    self.step()
                except Exception as exc:                      # keep the demo alive; show the error
                    self._event(-1, f"simulator error: {exc!r}")
            time.sleep(max(0.0, 1.0 / max(0.25, self.speed) - (time.time() - t0)))

    # ---- views ----------------------------------------------------------------------------
    def layout(self) -> dict:
        with self.lock:
            return {
                "w": self.rm.width, "h": self.rm.height, "name": self.layout_name,
                "nodes": [{"id": n.id, "x": int(n.x), "y": int(n.y), "kind": n.kind,
                           "corridor": n.corridor} for n in self.rm.nodes.values()],
            }

    def state(self) -> dict:
        w = self.world
        robots = []
        agents = getattr(self.policy, "agents", {})
        for r in range(self.n):
            h = self.heard.get(r, {})
            hello = h.get("hello")
            ag = agents.get(r)
            dec = getattr(self.policy, "last_decisions", {}).get(r)
            robots.append({
                "id": r, "alive": r in w.alive,
                "node": hello.node if hello is not None else w.pos[r],
                "true_node": w.pos[r],
                "battery": round(hello.battery if hello is not None else w.battery[r], 3),
                "load": bool(hello.load) if hello is not None else False,
                "task": hello.task_id if hello is not None else None,
                "goal": hello.goal if hello is not None else (self.policy.goal.get(r) if hasattr(self.policy, "goal") else None),
                "claim": hello.claim if hello is not None else None,
                "degraded": bool(hello.degraded) if hello is not None else False,
                "intent": h.get("intent", [])[:12],
                "last_heard": self.t - h.get("heard_t", self.t),
                "reason": dec.reason if dec is not None else "",
                "in_dead_zone": w.pos[r] in w.dead_zone,
                "priority": list(ag.bkey) if ag is not None and ag.bkey is not None else None,
            })
        lat = [self.done_at[k] - self.release[k] for k in self.done_at if k in self.release]
        recent = [k for k, v in self.done_at.items() if v > self.t - 60]
        st = self.policy.stats()
        new_events = self.events[self.event_cursor:]
        self.event_cursor = len(self.events)
        pk = sum(m[1] for m in self.msg_window) / max(1, len(self.msg_window))
        return {
            "t": self.t, "policy": self.policy_name, "speed": self.speed, "paused": self.paused,
            "robots": robots, "obstacles": sorted(w.obstacles), "dead_zone": sorted(w.dead_zone),
            "partition": {str(k): v for k, v in w.partition.items()}, "loss": w.loss,
            "kpi": {
                "orders": self.order_seq, "done": len(self.done_at),
                "open": self.order_seq - len(self.done_at),
                "throughput_per_min": len(recent),
                "mean_latency": round(sum(lat) / len(lat), 1) if lat else None,
                "collisions": self.collisions,
                "packets_per_tick": round(pk, 1),
                "bytes_per_robot_tick": round(st.get("bytes_per_robot_tick", 0.0), 1),
                "decision_ms_p99": round(st.get("decision_ms_p99", 0.0), 2),
                "pushes": st.get("pushes", 0), "swaps_cycles": st.get("cycles_broken", 0),
                "reassigned": st.get("market_releases", 0) + st.get("market_reopens", 0),
                "robots_alive": len(w.alive),
            },
            "events": new_events[-40:],
        }


def make_handler(sim: LiveSim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] in ("/", "/index.html"):
                self._send(200, (WEB / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif self.path == "/api/layout":
                self._send(200, json.dumps(sim.layout()).encode(), "application/json")
            elif self.path == "/api/state":
                with sim.lock:
                    self._send(200, json.dumps(sim.state()).encode(), "application/json")
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                q: queue.Queue = queue.Queue(maxsize=8)
                sim.subs.append(q)
                try:
                    while True:
                        state = q.get(timeout=15)
                        self.wfile.write(b"data: " + json.dumps(state).encode() + b"\n\n")
                        self.wfile.flush()
                except Exception:
                    pass
                finally:
                    if q in sim.subs:
                        sim.subs.remove(q)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                body = {}
            if self.path == "/api/chaos":
                sim.chaos(body.get("kind", ""), body.get("arg"))
            elif self.path == "/api/control":
                if "speed" in body:
                    sim.speed = float(body["speed"])
                if "paused" in body:
                    sim.paused = bool(body["paused"])
                if body.get("reset") or "policy" in body or "layout" in body or "robots" in body:
                    sim.reset(layout=body.get("layout"), robots=body.get("robots"),
                              seed=body.get("seed"), policy=body.get("policy"))
            self._send(200, b'{"ok":true}', "application/json")

    return Handler


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--robots", type=int, default=6)
    ap.add_argument("--layout", default="warehouse", choices=list(LAYOUTS))
    ap.add_argument("--speed", type=float, default=4.0, help="ticks (seconds of floor time) per real second")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--policy", default="sanchar", choices=["sanchar", "stop_and_wait"])
    a = ap.parse_args(argv)
    sim = LiveSim(a.layout, a.robots, a.seed, a.policy, a.speed)
    threading.Thread(target=sim.loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(sim))
    print(f"SANCHAR dashboard on http://127.0.0.1:{a.port}  (Ctrl+C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

"""Films the live dashboard for scenes 6-8 (choke points, blocked aisle, dead zone + robot failure).

The dashboard (sanchar/dashboard/server.py + web/index.html) runs unmodified. The simulation is
stepped by this script instead of its own timer thread, and the page's clock
(performance.now / requestAnimationFrame) is virtualised, so every frame is exact and
reproducible. The pallet and the dead zone are placed through the real UI (tool button, then a
click on the map); the robot kill uses the same chaos endpoint as the "Kill a robot" button.

Before filming, `plan()` replays seeds headlessly with the same action script and keeps the first
one where every event the narration mentions really happens on screen in time: a corridor-gate /
swap / push / yield decision, an order released because a pallet blocked its route, and the crashed
robot's order re-opened and won by another robot. Filming replays that exact pass and asserts the
robots made the same decisions. Zero collisions on every tick is asserted as well; the engine
counts collisions independently of the policy.
"""
from __future__ import annotations

import re
import threading
import time
from dataclasses import dataclass, field
from http.server import ThreadingHTTPServer

from sanchar.dashboard.server import LiveSim, make_handler

FPS = 30
BASE_RATE = 2.0            # ticks (1 s of floor time each) per video second
MAX_RATE = 18.0           # fast-forward cap while a crashed robot's order times out
MIN_MOVING = 2.5          # scene 8: robots moving per tick, on average, after the dead zone appears
MAX_ACK_HOLD = 1.0        # scene 8: robots holding for acknowledgements per tick, on average
FF_BADGE_RATE = 3.5
PREROLL = 30
CONFLICT_RE = r"corridor gate|swap:|push R|proactive yield|clear mouth|clear exit|wait-for cycle"
EVENTS_CARD = "aside.side > section:nth-of-type(3)"

VIRTUAL_CLOCK = """
(() => {
  let vt = 0; let q = [];
  Object.defineProperty(performance, "now", { value: () => vt, configurable: true });
  window.requestAnimationFrame = (cb) => { q.push(cb); return q.length; };
  window.cancelAnimationFrame = () => {};
  window.__advance = (ms) => { vt += ms; const cbs = q; q = []; for (const cb of cbs) { try { cb(vt); } catch (e) { console.error(e); } } };
})();
"""

# Compact the dashboard for a 1600x900 frame and hide controls the video never uses. Injected only
# into the filmed page; the dashboard file itself is unchanged.
DASH_CSS = """
label.clock, #pause, #reset, #theme { display: none !important; }
main { grid-template-columns: 960px minmax(0, 1fr) !important; gap: 14px !important; padding: 12px 16px !important; }
#ov-cap { left: 496px !important; max-width: 930px !important; }
.note { display: none !important; }
.card { padding: 10px 12px !important; }
.side { gap: 10px !important; }
.kpis { grid-template-columns: repeat(3, minmax(0, 1fr)) !important; gap: 6px !important; }
.kpi { padding: 6px 9px !important; }
.kpi .v { font-size: 21px !important; }
.kpi .l { font-size: 12.5px !important; }
.robots { max-height: none !important; gap: 5px !important; }
.robot { font-size: 14px !important; }
.events { font-size: 13.5px !important; max-height: 224px !important; overflow: hidden !important; }
"""

# The dashboard keeps its state in top-level `let`s; expose read-only views for the overlay layer.
GLOBALS_BRIDGE = """
Object.defineProperty(window, "__prevPos", { get: () => prevPos, configurable: true });
Object.defineProperty(window, "__curPos", { get: () => curPos, configurable: true });
window.__alpha = () => Math.min(1, (performance.now() - lastT) / tickMs);
window.__cell = () => cell;
window.__nodePt = (n) => { const r = document.getElementById("map").getBoundingClientRect(); const p = nodeXY(n); return [r.left + p[0], r.top + p[1]]; };
window.__center = (sel) => { const r = document.querySelector(sel).getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; };
"""


@dataclass
class Action:
    kind: str                  # block | deadzone | crash
    tick: int                  # applied at the start of this tick
    arg: int


@dataclass
class Plan:
    seed: int
    cand: int                  # index into the pallet candidate list
    anchors: list[tuple[float, int]]   # (video time, tick) the tick schedule passes through
    actions: list[Action]
    info: dict = field(default_factory=dict)
    events: list = field(default_factory=list)
    end_tick: int = 0


class Hooks:
    """How a pass touches the world. The default runs headless (seed search)."""

    def on_sim(self, sim: LiveSim) -> None:
        pass

    def step(self, sim: LiveSim) -> None:
        sim.step()

    def act(self, a: Action, sim: LiveSim) -> None:
        sim.chaos({"block": "toggle_block", "deadzone": "toggle_deadzone"}.get(a.kind, a.kind), a.arg)


class Director:
    def __init__(self, dash: dict, log=print, bench_p99_ms: float | None = None):
        """dash: {"dur", "cues", "scenes": [{"id", "start", "dur", "kw": {name: t}, "tag"}]} (video seconds).
        bench_p99_ms: the benchmark's decision-time p99, shown beside the live (filming laptop) figure."""
        self.dash = dash
        self.log = log
        self.bench_p99_ms = bench_p99_ms
        self.sc = {s["id"]: s for s in dash["scenes"]}

    # ---- timing -------------------------------------------------------------------------------
    def kw(self, scene: str, name: str) -> float:
        s = self.sc[scene]
        if name not in s["kw"]:
            raise KeyError(f"scene {scene} has no keyword {name!r}: {sorted(s['kw'])}")
        return s["start"] + s["kw"][name]

    def end(self, scene: str) -> float:
        return self.sc[scene]["start"] + self.sc[scene]["dur"]

    @staticmethod
    def tick_times(anchors: list[tuple[float, int]], last_tick: int) -> dict[int, float]:
        """Video time at which each tick is stepped: linear between anchors, BASE_RATE after the last."""
        out: dict[int, float] = {}
        for (t0, k0), (t1, k1) in zip(anchors, anchors[1:]):
            for k in range(k0, k1):
                out[k] = t0 + (k - k0) * (t1 - t0) / (k1 - k0)
        t_last, k_last = anchors[-1]
        for k in range(k_last, last_tick + 1):
            out[k] = t_last + (k - k_last) / BASE_RATE
        return out

    @staticmethod
    def tick_at(anchors, t: float) -> int:
        """First tick stepped at or after video time t, continuing at BASE_RATE from the last anchor."""
        t_last, k_last = anchors[-1]
        return k_last + max(1, int((t - t_last) * BASE_RATE + 0.999))

    @staticmethod
    def need_anchor(anchors, k: int, t_want: float) -> tuple[bool, float]:
        """Show tick k no later than t_want: fast-forward if the base rate would be late."""
        t_last, k_last = anchors[-1]
        if t_last + (k - k_last) / BASE_RATE <= t_want:
            return True, BASE_RATE
        rate = (k - k_last) / max(0.05, t_want - t_last)
        if rate <= MAX_RATE:
            anchors.append((t_want, k))
            return True, rate
        return False, rate

    # ---- where to act (chosen from what is on the floor at that moment) -----------------------
    @staticmethod
    def pallet_candidates(sim: LiveSim) -> list[int]:
        """Single-lane corridor cells on the planned route of a robot heading to a pickup."""
        rm, w, pol = sim.rm, sim.world, sim.policy
        occupied = set(w.pos.values())
        score: dict[int, float] = {}
        for rid in sorted(w.alive):
            m = pol.markets[rid]
            if m.my_task is None or m.tasks[m.my_task].status != "awarded":
                continue
            guide = list(getattr(pol.agents[rid], "guide", []) or [])
            left = rm.dist(w.pos[rid], m.tasks[m.my_task].pickup)
            for i, n in enumerate(guide[1:8], start=1):
                if rm.nodes[n].corridor is None or n in occupied or n in w.obstacles:
                    continue
                score[n] = score.get(n, 0.0) + 10.0 / (1 + left) + 1.0 / i
        return [n for n, _ in sorted(score.items(), key=lambda kv: (-kv[1], kv[0]))][:4]

    @staticmethod
    def deadzone_centres(sim: LiveSim, avoid: set[int]) -> list[int]:
        rm, w = sim.rm, sim.world
        alive = sorted(w.alive)

        def near(c):
            return sum(1 for r in alive if rm.dist(w.pos[r], c) <= 2)
        cands = [n for n, nd in rm.nodes.items() if nd.kind == "aisle" and n not in avoid and n not in w.obstacles]
        c1 = max(cands, key=lambda n: (near(n), -n))
        ring2 = [n for n in cands if rm.dist(n, c1) == 2]
        c2 = max(ring2, key=lambda n: (near(n), -n)) if ring2 else c1
        return [c1, c2] if c2 != c1 else [c1]

    @staticmethod
    def pick_victim(sim: LiveSim) -> tuple[int, str] | None:
        """A robot with an order, outside the dead zone; prefer one still heading to its pickup."""
        rm, w, pol = sim.rm, sim.world, sim.policy
        best = None
        for rid in sorted(w.alive):
            m = pol.markets[rid]
            if m.my_task is None or w.pos[rid] in w.dead_zone:
                continue
            rec = m.tasks[m.my_task]
            if rec.status not in ("awarded", "picked"):
                continue
            key = (0 if rec.status == "awarded" else 1, -rm.dist(w.pos[rid], rec.pickup), rid)
            if best is None or key < best[0]:
                best = (key, rid, m.my_task)
        return (best[1], best[2]) if best else None

    @staticmethod
    def central_robot(sim: LiveSim) -> int:
        rm, w = sim.rm, sim.world
        cx, cy = rm.width / 2, rm.height / 2
        return min(sorted(w.alive), key=lambda r: (abs(rm.pos(w.pos[r])[0] - cx) + abs(rm.pos(w.pos[r])[1] - cy), r))

    # ---- one scripted pass (seed search and filming share it) ---------------------------------
    def run_pass(self, seed: int, cand: int, hooks: Hooks | None = None) -> Plan | None:
        """Returns None if this seed does not show every narrated event in time."""
        hooks = hooks or Hooks()
        sim = LiveSim("warehouse", 6, seed, "sanchar", BASE_RATE)
        hooks.on_sim(sim)
        info: dict = {}
        actions: list[Action] = []
        where: dict[int, dict[int, int]] = {}         # tick -> robot positions after it
        flow: dict[int, tuple[int, int, int]] = {}    # tick -> (alive, moved, holding for acknowledgements)

        def run_to(k_end: int) -> None:               # step until sim.t == k_end
            while sim.t < k_end:
                before = dict(sim.world.pos)
                hooks.step(sim)
                k = sim.t - 1
                where[k] = dict(sim.world.pos)
                alive = sorted(sim.world.alive)
                holds = sum(1 for r in alive if (d := sim.policy.last_decisions.get(r)) is not None
                            and d.reason.startswith("hold: waiting for acks"))
                flow[k] = (len(alive), sum(1 for r in alive if sim.world.pos[r] != before[r]), holds)
                if sim.collisions:
                    raise RuntimeError(f"collision at tick {k} (seed {seed})")

        def act(kind: str, arg: int) -> None:
            a = Action(kind, sim.t, int(arg))
            hooks.act(a, sim)
            actions.append(a)

        def events_since(k0: int, pattern: str):
            return [(t, r, e) for t, r, e in sim.events if t >= k0 and re.search(pattern, e)]

        run_to(PREROLL)
        anchors: list[tuple[float, int]] = [(self.sc["06"]["start"], PREROLL)]

        # scene 6: conflict-resolution decisions appear before the narration names them
        k6_end = self.tick_at(anchors, self.end("06"))
        info["battery_robot"] = self.central_robot(sim)
        k_rules = self.tick_at(anchors, self.kw("06", "push, swap and yield")) - 1
        run_to(k_rules)
        conflict = events_since(PREROLL, CONFLICT_RE)
        if not [e for e in conflict if e[0] >= k_rules - 5]:     # fresh rows when the log is spotlit
            return None
        k_gate = self.tick_at(anchors, self.kw("06", "choke points")) - 1
        gate_like = [e for e in conflict if re.search("corridor gate|swap:", e[2])] or conflict
        ev = min(gate_like, key=lambda e: (abs(k_gate - e[0]), e[0]))
        node = where.get(ev[0], sim.world.pos)[ev[1]]
        cid = sim.rm.nodes[node].corridor
        for m in [node] + sorted(sim.rm.neighbors(node)):
            if sim.rm.nodes[m].corridor is not None:
                cid = sim.rm.nodes[m].corridor
                break
        info["gate_event"] = ev
        info["gate_nodes"] = sorted(n for n, nd in sim.rm.nodes.items() if cid is not None and nd.corridor == cid)
        info["plan_robot"] = ev[1]
        run_to(k6_end)
        anchors.append((self.end("06"), k6_end))

        # scene 7: a pallet on a corridor an order's route runs through
        t_pallet = self.kw("07", "pallet") + 0.25
        k_pallet = self.tick_at(anchors, t_pallet)
        run_to(k_pallet)
        cands = self.pallet_candidates(sim)
        if cand >= len(cands):
            return None
        pallet = cands[cand]
        anchors.append((t_pallet, k_pallet))
        act("block", pallet)
        info["pallet"] = pallet
        t_rel = self.kw("07", "released and re-auctioned") - 0.3
        k_limit = k_pallet + int(MAX_RATE * max(0.5, t_rel - t_pallet))
        released = None
        while released is None and sim.t <= k_limit:
            run_to(sim.t + 1)
            hit = events_since(k_pallet, r"released T\d+: route blocked")
            released = hit[0] if hit else None
        if released is None:
            return None
        info["released"] = released
        info["release_task"] = re.search(r"T\d+", released[2]).group(0)
        ok, rate7 = self.need_anchor(anchors, released[0] + 1, t_rel)
        if not ok:
            return None
        k7_end = self.tick_at(anchors, self.end("07"))
        run_to(max(sim.t, k7_end))
        rewon = [e for e in events_since(released[0], rf"claimed {info['release_task']} ") if e[1] != released[1]]
        info["rewon"] = rewon[0] if rewon else None

        # scene 8: Wi-Fi dead zone, then a robot fails
        t_dz = self.kw("08", "Wi-Fi dies") + 0.35
        k_dz = self.tick_at(anchors, t_dz)
        run_to(k_dz)
        anchors.append((t_dz, k_dz))
        zone = self.deadzone_centres(sim, avoid={pallet})
        for c in zone:
            act("deadzone", c)
        info["deadzone"] = zone
        t_crash = self.kw("08", "fails") + 0.15
        k_crash = self.tick_at(anchors, t_crash)
        run_to(k_crash)
        anchors.append((t_crash, k_crash))
        vic = self.pick_victim(sim)
        if vic is None:
            return None
        victim, task = vic
        act("crash", victim)
        info.update(victim=victim, victim_task=task)
        reopened = won = None
        while won is None and sim.t <= k_crash + 60:
            run_to(sim.t + 1)
            if reopened is None:
                hit = events_since(k_crash, rf"re-opened {task}: owner R{victim} silent")
                reopened = hit[0] if hit else None
            if reopened is not None:
                hit = [e for e in events_since(reopened[0], rf"claimed {task} ") if e[1] != victim]
                won = hit[0] if hit else None
        if reopened is None or won is None:
            return None
        info.update(reopened=reopened, won=won)
        ok, rate = self.need_anchor(anchors, reopened[0] + 1, self.kw("08", "re-opens") - 0.25)
        if not ok:
            return None
        info["ff_rate"] = rate
        info["ff"] = [t_crash, anchors[-1][0]] if rate > FF_BADGE_RATE else None
        ok, _ = self.need_anchor(anchors, won[0] + 1, self.kw("08", "another robot wins it") + 0.3)
        if not ok:
            return None
        run_to(max(sim.t, self.tick_at(anchors, self.end("08"))))
        info["rate7"] = rate7
        # The narration says robots keep talking in the dead zone and route around the failed robot, so
        # the fleet must visibly flow while those lines play: after the dead zone appears (before the
        # crash), and after the order re-opens. While a robot has just gone silent, peers hold for its
        # acknowledgement by design; the fast-forward covers that stretch.
        def rate(k0: int, k1: int) -> tuple[float, float]:
            w = [flow[k] for k in range(k0, k1) if k in flow]
            return (round(sum(m for _, m, _ in w) / max(1, len(w)), 2),
                    round(sum(h for _, _, h in w) / max(1, len(w)), 2))
        info["flow"] = (rate(k_dz, k_crash), rate(reopened[0] + 1, sim.t))
        if any(m < MIN_MOVING or h > MAX_ACK_HOLD for m, h in info["flow"]):
            return None
        return Plan(seed, cand, anchors, actions, info, [tuple(e) for e in sim.events], sim.t)

    def plan(self, seeds=range(1, 61), first: tuple[int, int] | None = None) -> Plan:
        """First seed (and pallet candidate) that shows every narrated event; `first` is tried before the rest."""
        t0 = time.time()
        order = [(s, c) for s in seeds for c in range(4)]
        if first is not None:
            order.insert(0, tuple(first))
        for seed, cand in order:
            try:
                p = self.run_pass(seed, cand)
            except RuntimeError as exc:
                raise SystemExit(f"SAFETY: {exc}")
            if p is not None:
                self.log(f"  director: seed {seed}, pallet candidate {cand}, fast-forward "
                         f"x{p.info['ff_rate'] / BASE_RATE:.1f}, robots moving/holding per tick after the dead "
                         f"zone {p.info['flow'][0]} and after the re-open {p.info['flow'][1]} "
                         f"({time.time() - t0:.0f} s search)")
                return p
        raise SystemExit("director: no seed showed every narrated event in time; widen the search")

    # ---- overlays and cursor ------------------------------------------------------------------
    def ui_schedule(self, p: Plan) -> tuple[list[dict], list[list[dict]]]:
        """Real UI clicks (time, what) and the matching cursor key-frame sequences."""
        tt = self.tick_times(p.anchors, p.end_tick + 2)
        clicks: list[dict] = []
        cursor: list[list[dict]] = []
        by_kind: dict[str, list[Action]] = {}
        for a in p.actions:
            by_kind.setdefault(a.kind, []).append(a)
        for kind, tool in (("block", "block"), ("deadzone", "deadzone")):
            acts = by_kind.get(kind, [])
            if not acts:
                continue
            k = acts[0].tick
            lo, hi = tt[k - 1], tt[k]
            node_t = [hi - 0.08 - 0.22 * (len(acts) - 1 - i) for i in range(len(acts))]
            if node_t[0] <= lo + 0.02:
                raise RuntimeError(f"no room to click {kind} between ticks {k - 1} and {k}")
            tool_t = node_t[0] - 0.75
            sel = f'#tool button[data-t="{tool}"]'
            clicks.append({"t": tool_t, "sel": sel})
            clicks += [{"t": t, "node": a.arg, "kind": kind} for t, a in zip(node_t, acts)]
            cursor.append([{"t": tool_t - 0.7, "sel": "#map"}, {"t": tool_t, "sel": sel, "click": True}]
                          + [{"t": t, "node": a.arg, "click": True} for t, a in zip(node_t, acts)])
        for a in by_kind.get("crash", []):
            t = tt[a.tick] - 0.05
            cursor.append([{"t": t - 0.9, "sel": "#map"}, {"t": t, "sel": "#kill", "click": True}])
        return clicks, cursor

    def overlays(self, p: Plan) -> list[dict]:
        I, k = p.info, self.kw
        ov: list[dict] = []
        for sid in ("06", "07", "08"):
            s = self.sc[sid]
            if s.get("tag"):
                ov.append({"type": "tag", "text": s["tag"], "at": s["start"] + 0.05, "until": self.end(sid) - 0.15, "fade": 0.2})
        # scene 6
        ov.append({"type": "callout", "robot": I["battery_robot"], "text": "live position + battery ring",
                   "at": k("06", "positions and battery"), "until": k("06", "only listens") - 0.1, "dx": 80, "dy": -80})
        ov.append({"type": "box", "sel": "main > section.card > h2", "text": "a passive listener: it hears only the robots' own broadcasts",
                   "labelPos": "below", "at": k("06", "only listens"), "until": k("06", "broadcast plan") - 0.1, "pad": 4})
        ov.append({"type": "callout", "robot": I["plan_robot"], "text": "each line = that robot's broadcast plan (Intent)",
                   "at": k("06", "broadcast plan"), "until": k("06", "choke points") - 0.1, "dx": 80, "dy": 80})
        if I["gate_nodes"]:
            ov.append({"type": "box", "nodes": I["gate_nodes"], "text": "single-lane corridor: one direction at a time",
                       "at": k("06", "choke points"), "until": k("06", "push, swap and yield") - 0.1, "pad": 3})
        ov.append({"type": "spot", "sel": EVENTS_CARD, "at": k("06", "push, swap and yield"), "until": self.end("06") - 0.05, "pad": 4})
        ov.append({"type": "mark", "sel": "#events > div", "re": CONFLICT_RE,
                   "at": k("06", "push, swap and yield"), "until": self.end("06") - 0.05})
        # scene 7
        ov.append({"type": "callout", "node": I["pallet"], "text": "pallet: the robot that sees it broadcasts an ObstacleDelta",
                   "at": k("07", "obstacle report"), "until": k("07", "released and re-auctioned") - 0.1, "dx": 70, "dy": -90})
        rel_robot, task = I["released"][1], I["release_task"]
        t_won = None
        if I.get("rewon"):
            tt = self.tick_times(p.anchors, p.end_tick + 2)
            t_won = max(tt.get(I["rewon"][0], 1e9), k("07", "CRDT task board"))
            if t_won > self.end("07") - 1.0:
                t_won = None
        ov.append({"type": "callout", "robot": rel_robot, "text": f"R{rel_robot} re-routes; its order {task} is released",
                   "at": k("07", "re-route"), "until": (t_won or self.end("07")) - 0.1, "dx": -80, "dy": 90})
        ov.append({"type": "box", "sel": "#kpis > .kpi:nth-child(8)", "text": "orders re-assigned, no dispatcher",
                   "labelPos": "below", "at": k("07", "released and re-auctioned"), "until": self.end("07") - 0.05, "pad": 3})
        if t_won is not None:
            ov.append({"type": "callout", "robot": I["rewon"][1], "text": f"R{I['rewon'][1]} wins {task} in the re-auction",
                       "at": t_won, "until": self.end("07") - 0.05, "dx": 80, "dy": -80})
        ov.append({"type": "mark", "sel": "#events > div", "re": rf"released {task}|claimed {task} ",
                   "at": k("07", "re-route"), "until": self.end("07") - 0.05})
        # scene 8
        zone = sorted({c for c in I["deadzone"]} | {n for c in I["deadzone"] for n in self._neighbors(c)})
        ov.append({"type": "box", "nodes": zone, "text": "Wi-Fi dead zone: robots still talk peer-to-peer at short range",
                   "at": k("08", "short-range links"), "until": k("08", "fails") - 0.3, "pad": 2})
        ov.append({"type": "callout", "robot": I["victim"], "text": f"R{I['victim']} powered off",
                   "at": k("08", "fails") + 0.3, "until": k("08", "re-opens") - 0.1, "dx": 80, "dy": -80})
        if I["ff"]:
            ov.append({"type": "badge", "text": f"▶▶  fast-forward ×{round(I['ff_rate'] / BASE_RATE)}",
                       "at": I["ff"][0] + 0.2, "until": I["ff"][1], "fade": 0.2})
        ov.append({"type": "spot", "sel": EVENTS_CARD, "at": k("08", "re-opens"), "until": k("08", "zero") - 0.4, "pad": 4})
        ov.append({"type": "mark", "sel": "#events > div", "re": rf"re-opened {I['victim_task']}|claimed {I['victim_task']} ",
                   "at": k("08", "re-opens"), "until": k("08", "zero") - 0.4})
        ov.append({"type": "spot", "sel": "#kpis > .kpi:nth-child(1)", "at": k("08", "zero") - 0.2, "until": self.end("08") + 5, "pad": 4})
        # keyword strips under the map legend (on screen only; the narration is unchanged)
        s6 = self.sc["06"]["start"]
        ov.append({"type": "chips", "at": s6 + 0.3, "until": self.end("06") - 0.1, "fade": 0.2, "items": [
            {"text": "Live multi-robot simulation", "at": s6 + 0.4},
            {"text": "Dashboard fed by Server-Sent Events", "at": k("06", "only listens")},
            {"text": "Explainable: every decision logged with its reason", "at": k("06", "push, swap and yield")}]})
        ov.append({"type": "chips", "at": k("07", "released and re-auctioned") - 0.1, "until": self.end("07") - 0.1, "fade": 0.2,
                   "items": [
                       {"text": "Task allocation: the pickup point is re-assigned by auction", "at": k("07", "released and re-auctioned")},
                       {"text": "CRDT merge: commutative, associative, idempotent", "at": k("07", "CRDT task board")},
                       {"text": "No Raft or Paxos leader", "at": k("07", "CRDT task board") + 0.6}]})
        ov.append({"type": "chips", "at": k("08", "short-range links") - 0.1, "until": self.end("08") + 5, "fade": 0.2,
                   "items": [{"text": "Fault-tolerant: graceful degradation, not a stop", "at": k("08", "short-range links")}]})
        # The live decision-time tile runs on the laptop that is also rendering this video, so it reads
        # higher than the benchmark; say so beside it (in the empty grid cells right of the last tile),
        # except while scene 7's "orders re-assigned" label sits in those cells.
        if self.bench_p99_ms is not None:
            note = {"type": "note", "sel": "#kpis > .kpi:last-child", "within": "#kpis", "fade": 0.2,
                    "text": f"↖ live, on the recording laptop\nbenchmark p99: {self.bench_p99_ms:g} ms at most"}
            ov.append({**note, "at": self.sc["06"]["start"] + 0.3, "until": k("07", "released and re-auctioned") - 0.3})
            ov.append({**note, "at": self.end("07") + 0.3, "until": self.end("08") + 5})
        return ov

    def _neighbors(self, n: int) -> list[int]:
        if not hasattr(self, "_rm"):
            self._rm = LiveSim("warehouse", 6, 1, "sanchar", BASE_RATE).rm
        return list(self._rm.neighbors(n))


class Filmer(Hooks):
    """Replays a Plan in headless Edge and hands every frame to emit(scene_id, jpeg_bytes)."""

    def __init__(self, director: Director, plan: Plan, browser, css: str, js: str, emit):
        self.d, self.p, self.browser, self.css, self.js, self.emit = director, plan, browser, css, js, emit
        self.ticks = Director.tick_times(plan.anchors, plan.end_tick + 2)
        self.clicks, self.cursor = director.ui_schedule(plan)
        self.frames_total = round(director.dash["dur"] * FPS)
        self.frame = 0
        self.page = None
        self.sim: LiveSim | None = None
        self.srv = None

    # Hooks -------------------------------------------------------------------------------------
    def on_sim(self, sim: LiveSim) -> None:
        self.sim = sim
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(sim))
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        pg = self.page = self.browser.new_page(viewport={"width": 1600, "height": 900}, device_scale_factor=1.2,
                                               color_scheme="dark")
        pg.add_init_script(VIRTUAL_CLOCK)
        pg.goto(f"http://127.0.0.1:{self.srv.server_address[1]}/")
        deadline = time.time() + 20
        while not sim.subs and time.time() < deadline:
            time.sleep(0.02)
        if not sim.subs:
            raise RuntimeError("dashboard page did not connect")
        pg.add_style_tag(content=self.css)
        pg.add_style_tag(content=DASH_CSS)
        pg.add_script_tag(content=GLOBALS_BRIDGE)
        pg.add_script_tag(content=self.js)
        # the compact layout narrows the map: let the dashboard re-size its canvas so cells stay square
        pg.evaluate("() => { window.dispatchEvent(new Event('resize')); return true; }")
        if pg.evaluate("() => { const c = document.getElementById('map'); return Math.abs(c.clientWidth - c.width / devicePixelRatio) > 2; }"):
            raise RuntimeError("dashboard canvas did not re-size to the video layout")
        pg.evaluate("(data) => V.init(data)", {
            "dur": self.d.dash["dur"], "cues": self.d.dash["cues"], "words": [], "kw": {},
            "overlays": self.d.overlays(self.p), "cursor": self.cursor})

    def step(self, sim: LiveSim) -> None:
        k = sim.t
        if k >= PREROLL:
            self.render_until(self.ticks[k])
            nxt, cur = self.ticks.get(k + 1), self.ticks[k]
            sim.speed = BASE_RATE if nxt is None else max(0.5, 1.0 / max(1e-3, nxt - cur))
        else:
            sim.speed = BASE_RATE
        sim.step()
        self.page.wait_for_function(
            "(n) => (document.getElementById('clock').textContent || '').startsWith('t = ' + n + ' ')",
            arg=sim.t, polling=5, timeout=20000)

    def act(self, a: Action, sim: LiveSim) -> None:
        if a.kind == "crash":
            self.render_until(self.ticks[a.tick] - 1e-6)
            sim.chaos("crash", a.arg)
            return
        # pallet / dead zone: the page clicked it already; make sure the fleet got exactly that
        self.render_until(self.ticks[a.tick] - 1e-6)
        want = ({"block": "toggle_block", "deadzone": "toggle_deadzone"}[a.kind], a.arg)
        if not any(k == want[0] and int(v) == a.arg for k, v in sim.pending_chaos):
            raise RuntimeError(f"UI click for {a.kind} at node {a.arg} did not reach the fleet: {sim.pending_chaos}")

    # frames ------------------------------------------------------------------------------------
    def render_until(self, t_video: float) -> None:
        while self.frame < self.frames_total and self.frame / FPS < t_video - 1e-9:
            tv = self.frame / FPS
            while self.clicks and self.clicks[0]["t"] <= tv:
                self._click(self.clicks.pop(0))
            self.page.evaluate("([dt, t]) => { window.__advance(dt); return V.setFrame(t); }", [1000.0 / FPS, tv])
            self.emit(self._scene_of(tv), self.page.screenshot(type="jpeg", quality=90))
            self.frame += 1

    def _click(self, c: dict) -> None:
        pg = self.page
        if "sel" in c:
            x, y = pg.evaluate("(s) => window.__center(s)", c["sel"])
            pg.mouse.click(x, y)
            return
        before = len(self.sim.pending_chaos)
        x, y = pg.evaluate("(n) => window.__nodePt(n)", c["node"])
        pg.mouse.click(x, y)
        deadline = time.time() + 5
        while len(self.sim.pending_chaos) <= before and time.time() < deadline:
            time.sleep(0.005)
        got = self.sim.pending_chaos[-1] if len(self.sim.pending_chaos) > before else None
        if got is None or int(got[1]) != int(c["node"]):
            raise RuntimeError(f"map click for node {c['node']} reached the fleet as {got}")

    def _scene_of(self, tv: float) -> str:
        for s in self.d.dash["scenes"]:
            if s["start"] - 1e-9 <= tv < s["start"] + s["dur"] - 1e-9:
                return s["id"]
        return self.d.dash["scenes"][-1]["id"]

    def film(self) -> None:
        try:
            replay = self.d.run_pass(self.p.seed, self.p.cand, hooks=self)
            if replay is None:
                raise RuntimeError("filming pass diverged from the plan")
            self.render_until(self.d.dash["dur"] + 1)
            if replay.events != self.p.events:
                raise RuntimeError("the filmed robots made different decisions from the planned run")
            if self.sim.collisions:
                raise RuntimeError("collision during filming")
        finally:
            if self.srv is not None:
                self.srv.shutdown()
            if self.page is not None:
                self.page.close()

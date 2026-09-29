"""Narration, captions and visual plan for the SANCHAR demo video.

Every number comes from bench/results/ (never hand-typed).

Narration mini-syntax, per sentence:
    {text}             keyword: highlighted in the caption, and slides can react when it is spoken
    [[shown||spoken]]  what the caption shows vs what the voice says ("A*" vs "A-star")
    " / "              caption break inside a sentence
"""
from __future__ import annotations

import html
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "bench/results"


# ---- numbers ----------------------------------------------------------------------------------
def load_summary() -> dict:
    rows = json.loads((RESULTS / "summary.json").read_text(encoding="utf-8"))
    return {r["family"]: r for r in rows}


def red(s: dict, key: str = "makespan_reduction_pct_both_finished") -> float:
    v = s[key][0]
    return round(v if v == v else s["makespan_reduction_pct"][0], 1)


def ci(s: dict, key: str = "makespan_reduction_pct_both_finished") -> float:
    v = s[key][1]
    return round(v if v == v else s["makespan_reduction_pct"][1], 1)


def numbers() -> dict:
    S = load_summary()
    kill = json.loads((RESULTS / "runner_kill.json").read_text(encoding="utf-8"))
    kb = [s["bytes_per_robot_tick"] / 1000 for s in S.values() if s.get("bytes_per_robot_tick")]
    p99 = [s["decision_ms_p99"] for s in S.values() if s.get("decision_ms_p99")]
    cross = S["intersection_cross"]
    return {
        "families": len(S),
        "seeds": min(s["seeds"] for s in S.values()),
        "runs": sum(s["seeds"] for s in S.values()),
        "collisions": sum(s["sanchar_collisions"] for s in S.values()),
        "unfinished": sum(s["sanchar_unfinished"] for s in S.values()),
        "r6": round(red(S["warehouse_overlap"])),
        "r6_exact": red(S["warehouse_overlap"]), "r6_ci": ci(S["warehouse_overlap"]),
        "r6_better": S["warehouse_overlap"]["seeds_sanchar_better"],
        "r12": round(red(S["warehouse_12"])),
        "rmarket": round(red(S["market_warehouse"])),
        "weak": round(-red(S["corridor_swap"])),
        "cross_gridlocked": cross["baseline_gridlocked_seeds"], "cross_seeds": cross["seeds"],
        "kb_lo": max(1, round(min(kb))), "kb_hi": round(max(kb)),
        "p99_ms": round(max(p99), 1), "p99_ms_median": round(sorted(p99)[len(p99) // 2], 2),
        "kill": kill,
    }


# ---- parsing ----------------------------------------------------------------------------------
@dataclass
class Seg:
    show: str
    say: str
    kw: str | None = None          # keyword name (its shown text) when highlighted


@dataclass
class Cue:
    segs: list[Seg]

    @property
    def show(self) -> str:
        return "".join(s.show for s in self.segs).strip()

    @property
    def say(self) -> str:
        return "".join(s.say for s in self.segs).strip()

    @property
    def html(self) -> str:
        out = []
        for s in self.segs:
            t = html.escape(s.show)
            out.append(f'<b class="kw">{t}</b>' if s.kw else t)
        return "".join(out).strip()


@dataclass
class Sentence:
    cues: list[Cue]
    pause_after: float = 0.0

    @property
    def say(self) -> str:
        return " ".join(c.say for c in self.cues)


@dataclass
class Scene:
    id: str
    title: str
    kind: str                       # "slide" | "dash"
    sentences: list[Sentence]
    tag: str = ""                   # e.g. "PS requirement 2 · conflict resolution"
    extra: dict = field(default_factory=dict)

    @property
    def cues(self) -> list[Cue]:
        return [c for s in self.sentences for c in s.cues]

    @property
    def say(self) -> str:
        return " ".join(s.say for s in self.sentences)

    @property
    def show(self) -> str:
        return " ".join(c.show for c in self.cues)


def parse_sentence(src: str, pause_after: float = 0.0) -> Sentence:
    cues: list[Cue] = []
    segs: list[Seg] = []
    kw_show: list[str] | None = None
    i = 0
    buf_show, buf_say = [], []

    def flush(kw: str | None = None) -> None:
        if buf_show or buf_say:
            segs.append(Seg("".join(buf_show), "".join(buf_say), kw))
            buf_show.clear()
            buf_say.clear()

    while i < len(src):
        if src.startswith(" / ", i):
            flush(None if kw_show is None else "".join(kw_show))
            cues.append(Cue(segs))
            segs = []
            i += 3
        elif src.startswith("[[", i):
            j = src.index("]]", i)
            shown, spoken = src[i + 2:j].split("||")
            buf_show.append(shown)
            buf_say.append(spoken)
            if kw_show is not None:
                kw_show.append(shown)
            i = j + 2
        elif src[i] == "{":
            flush()
            kw_show = []
            i += 1
        elif src[i] == "}":
            flush("".join(kw_show))
            kw_show = None
            i += 1
        else:
            buf_show.append(src[i])
            buf_say.append(src[i])
            if kw_show is not None:
                kw_show.append(src[i])
            i += 1
    flush()
    cues.append(Cue(segs))
    return Sentence(cues, pause_after)


def P(n: float) -> str:
    """A percentage: shown as '25%', spoken as '25 percent'."""
    v = f"{n:g}"
    return f"[[{v}%||{v} percent]]"


# ---- the script -------------------------------------------------------------------------------
# Play order: the live dashboard (06-08, filmed as one block) right after the problem, then how it
# works. Ids stay as recorded (voice/sceneNN.m4a).
ORDER = ["01", "02", "06", "07", "08", "03", "04", "05", "09", "10"]


def build_scenes(n: dict) -> list[Scene]:
    """The scenes in play order (ORDER)."""
    by_id = {s.id: s for s in _scenes(n)}
    return [by_id[i] for i in ORDER]


def _scenes(n: dict) -> list[Scene]:
    k = n["kill"]
    every_order = k["orders_done"] == k["orders"]
    gridlock_all = n["cross_gridlocked"] == n["cross_seeds"]
    gridlock = ("every time" if gridlock_all
                else f"in {n['cross_gridlocked']} of {n['cross_seeds']} runs")
    ms = ("under a millisecond" if n["p99_ms"] < 1.0
          else f"under {math.ceil(n['p99_ms'])} milliseconds")

    def S(*srcs, pauses: dict | None = None) -> list[Sentence]:
        pauses = pauses or {}
        return [parse_sentence(s, pauses.get(i, 0.0)) for i, s in enumerate(srcs)]

    SANCHAR = "[[SANCHAR||Sanchar]]"
    return [
        Scene("01", "Title", "slide", S(
            f"This is {{{SANCHAR}}}, for problem statement {{[[SIH26123||S-I-H two six one two three]]}} / "
            "from {Bharat Electronics Limited}:",
            "a fleet of autonomous mobile robots with {no master}. / No server, no leader, no broker.",
        )),
        Scene("02", "The problem", "slide", S(
            "Today, one {central server} plans the whole fleet over Wi-Fi.",
            "That means {network latency}, {Wi-Fi dead zones} / and a {single point of failure}: "
            "when the link drops, every robot stops.",
            "The targets: {zero inter-robot collisions}, / "
            f"and at least {{{P(20)} faster than stop-and-wait}}.",
        )),
        Scene("03", "One robot, one edge node", "slide", S(
            "Every robot runs the same {edge node}, / designed for {Raspberry Pi or Jetson} class hardware.",
            "It shares its {localization and intent} with peers, / bids for orders in a {decentralized auction}, / "
            "and does {multi-agent path planning} on board:",
            "{space-time [[A*||A-star]]} for its route, / "
            "and {[[Guided-PIBT||Guided P-I-B-T]]}, {priority inheritance with backtracking}, for each step.",
        )),
        Scene("04", "Peer-to-peer protocol", "slide", S(
            "Every round, each robot broadcasts a {Hello}: / "
            "its position, the node it claims next, its priority, battery, and which peers it heard.",
            "An {Intent} carries its next ten path nodes.",
            "Every packet is signed with {[[HMAC-SHA256||H-mac SHA two fifty-six]]} / and checked against an {anti-replay window}, / "
            "and robots {relay} each other's messages up to {two hops}, across dead zones.",
        ), tag="PS requirement 1 · Decentralized communication"),
        Scene("05", "Motion model and safety rule", "slide", S(
            "Motion uses a {discrete-time kinematic model}: / "
            "{one metre per second} on a one-metre roadmap.",
            "Each one-second tick has a {claim round} and a {move round}.",
            "A robot enters a node only if it claimed it, / every peer {acknowledged} hearing it, / "
            "and no peer's position, claim or {reach set} conflicts.",
            "So a lost packet can only cause a wait, never a collision.",
        ), tag="PS requirement 2 · Conflict resolution"),
        Scene("06", "Live dashboard and choke points", "dash", S(
            "The dashboard shows live {positions and battery}, / and it {only listens} to the robots' radio.",
            "Each line is a robot's {broadcast plan}.",
            "At single-lane {choke points}, a {corridor gate} passes one direction at a time, / "
            "and {push, swap and yield} rules with a {wait-for cycle breaker} / resolve {deadlocks} in real time.",
        ), tag="PS requirement 2 · Conflict resolution"),
        Scene("07", "Blocked aisle", "dash", S(
            "A {pallet} blocks an aisle.",
            "The robot that sees it broadcasts an {obstacle report}, / robots {re-route}, / "
            "and blocked orders are {released and re-auctioned} on a {[[CRDT||C-R-D-T]] task board}.",
            pauses={0: 0.5},
        ), tag="PS requirement 3 · Task allocation & re-routing"),
        Scene("08", "Dead zone and robot failure", "dash", S(
            "Now the {Wi-Fi dies} over part of the floor, / and nearby robots keep talking over {short-range links}.",
            "Then a robot {fails}.",
            "Its order {re-opens}, {another robot wins it}, / and the rest route around it.",
            "Collisions: still {zero}.",
            pauses={1: 1.8},                  # the video fast-forwards here until the order re-opens
        ), tag="PS requirement 3 · Task allocation & re-routing"),
        Scene("09", "Benchmark results", "slide", S(
            f"Across {{{n['families']} scenarios}}, {n['seeds']} seeds each, "
            f"{SANCHAR} had {{zero collisions in all {n['runs']} runs}}.",
            f"Measured with {{[[95% confidence intervals||95 percent confidence intervals]]}}, / "
            f"{{six robots}} on overlapping paths finish {{{P(n['r6'])} faster}} than stop-and-wait, / "
            f"{{twelve robots}} {P(n['r12'])}, and {{live orders}} {P(n['rmarket'])}.",
            f"At a single-lane {{intersection}}, stop-and-wait gridlocked {gridlock}.",
            f"Our {{weak case}}: an isolated two-way chokepoint, / where central zone control is {P(n['weak'])} faster.",
        )),
        Scene("10", "Proof and next steps", "slide", S(
            "We also run {one process per robot over real [[UDP||U-D-P]]} and {kill one mid-run}: / "
            + ("the rest still deliver every order." if every_order
               else f"the rest deliver {k['orders_done']} of {k['orders']} orders."),
            f"Each decision takes {{{ms}}} on a laptop, / "
            f"with {{{n['kb_lo']} to {n['kb_hi']} kilobytes}} of radio traffic per robot per second.",
            "Next, at the finale: {continuous velocity and acceleration profiles}, / "
            "{direction batching} at choke points, and tests on {Raspberry Pi hardware}.",
            f"{{{SANCHAR}}}: a fleet that keeps moving when the network dies.",
        )),
    ]


def keywords(scene: Scene) -> list[str]:
    return [s.kw for c in scene.cues for s in c.segs if s.kw]


def check_numbers(scenes: list[Scene], n: dict) -> None:
    """Every figure in the narration must be the one in bench/results (guards against drift)."""
    text = " ".join(s.show for s in scenes)
    must = [f"{n['r6']}%", f"{n['r12']}%", f"{n['rmarket']}%", f"{n['weak']}%",
            f"all {n['runs']} runs", f"{n['families']} scenarios", f"{n['seeds']} seeds",
            f"{n['kb_lo']} to {n['kb_hi']} kilobytes"]
    missing = [m for m in must if m not in text]
    if missing:
        raise SystemExit(f"narration lost figures: {missing}")
    if n["collisions"] != 0 or n["unfinished"] != 0:
        raise SystemExit("summary.json reports SANCHAR collisions or unfinished runs; the script says zero")


def words(text: str) -> list[str]:
    return text.split()


def norm(w: str) -> str:
    return re.sub(r"[^a-z0-9*%-]", "", w.lower())

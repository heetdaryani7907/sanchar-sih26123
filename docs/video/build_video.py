"""Build the SANCHAR demo video (about 3 minutes, 1920x1080, burned-in captions) for the SIH26123
idea submission. Every number is read from bench/results/ (never hand-typed).

    python docs/video/build_video.py [--team-name "..."] [--team-id "..."]
                                     [--voice tts|mine|none] [--tts-voice en-IN-NeerjaNeural] [--check]

--voice tts   (default) Indian-English neural voice via edge-tts; captions timed to its words.
--voice mine  your own recordings, docs/video/voice/scene01.m4a ... scene10.m4a (see voice/README.md);
              scenes stretch to your pace and captions follow. Missing scenes fall back to tts.
--voice none  silent, paced at 140 words per minute (a teleprompter to record against).

Outputs in docs/video/: SANCHAR_SIH26123_demo.mp4, SCRIPT.md, captions.srt.
Needs: pip install playwright edge-tts imageio-ffmpeg numpy (uses the installed Microsoft Edge).
"""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import html
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

if os.environ.get("PYTHONHASHSEED") != "0":        # reproducible robot decisions across rebuilds
    sys.exit(subprocess.call([sys.executable, *sys.argv], env=dict(os.environ, PYTHONHASHSEED="0")))

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT), str(HERE)]

from scenes import Scene, build_scenes, check_numbers, load_summary, norm, numbers, red, ci  # noqa: E402

FPS = 30
SR = 48000
SPF = SR // FPS                   # audio samples per video frame
BUILD = HERE / "build"
OUT_MP4 = HERE / "SANCHAR_SIH26123_demo.mp4"
LEAD, GAP, TAIL = 0.4, 0.2, 0.35
SCENE_LEAD = {"01": 0.8, "06": 0.6}
SCENE_TAIL = {"10": 1.6, "08": 0.8}
AUDIO_EXT = (".m4a", ".mp3", ".wav", ".ogg", ".aac", ".flac", ".webm", ".mp4")
MAX_TEMPO = 1.4


def log(msg: str) -> None:
    print(msg, flush=True)


def ffmpeg() -> str:
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def decode(src: str | Path, filt: str | None = None) -> np.ndarray:
    cmd = [ffmpeg(), "-v", "error", "-i", str(src)]
    if filt:
        cmd += ["-af", filt]
    cmd += ["-ac", "1", "-ar", str(SR), "-f", "s16le", "-"]
    return np.frombuffer(subprocess.run(cmd, check=True, capture_output=True).stdout, dtype=np.int16)


# ---- audio: one (pcm, word_times) per sentence ---------------------------------------------------
async def _tts(text: str, voice: str, rate: str, mp3: Path, meta: Path) -> None:
    import edge_tts
    com = edge_tts.Communicate(text, voice, rate=rate, boundary="WordBoundary")
    bounds, audio = [], bytearray()
    async for ch in com.stream():
        if ch["type"] == "audio":
            audio += ch["data"]
        elif ch["type"] == "WordBoundary":
            bounds.append([ch["offset"] / 1e7, ch["duration"] / 1e7, ch["text"]])
    mp3.write_bytes(bytes(audio))
    meta.write_text(json.dumps(bounds), encoding="utf-8")


def tts_sentence(text: str, voice: str, rate: str) -> tuple[np.ndarray, list[float]]:
    d = BUILD / "tts"
    d.mkdir(parents=True, exist_ok=True)
    key = hashlib.sha1(f"{voice}|{rate}|{text}".encode()).hexdigest()[:16]
    mp3, meta = d / f"{key}.mp3", d / f"{key}.json"
    if not mp3.exists() or not meta.exists():
        for attempt in range(4):
            try:
                asyncio.run(_tts(text, voice, rate, mp3, meta))
                break
            except Exception as exc:                  # transient network errors
                if attempt == 3:
                    raise SystemExit(f"edge-tts failed ({exc!r}); try --voice none or check the internet")
                time.sleep(2 + 3 * attempt)
    pcm = decode(mp3)
    bounds = json.loads(meta.read_text(encoding="utf-8"))
    words = text.split()
    if not bounds:
        return pcm, proportional(words, 0.0, len(pcm) / SR)
    a = max(0.0, bounds[0][0] - 0.06)
    b = min(len(pcm) / SR, bounds[-1][0] + bounds[-1][1] + 0.16)
    pcm = pcm[int(a * SR):int(b * SR)]
    if len(bounds) == len(words):
        return pcm, [bd[0] - a for bd in bounds]
    return pcm, proportional(words, bounds[0][0] - a, bounds[-1][0] + bounds[-1][1] - a)


def proportional(words: list[str], t0: float, t1: float) -> list[float]:
    """Word start times spread by character count (plus a little per word)."""
    w = [len(x) + 2 for x in words]
    tot, acc, out = sum(w), 0, []
    for x in w:
        out.append(t0 + (t1 - t0) * acc / tot)
        acc += x
    return out


def voiced_runs(pcm: np.ndarray) -> list[tuple[float, float]]:
    """Speech runs (seconds) from 20 ms frame energy."""
    hop = SR // 50
    n = len(pcm) // hop
    if n == 0:
        return []
    fr = pcm[: n * hop].astype(np.float32).reshape(n, hop)
    db = 20 * np.log10(np.sqrt((fr ** 2).mean(axis=1)) + 1e-3)
    thr = max(np.percentile(db, 10) + 10, np.percentile(db, 98) - 32)
    v = db > thr
    runs, i = [], 0
    while i < n:
        if v[i]:
            j = i
            while j < n and v[j]:
                j += 1
            runs.append([i / 50, j / 50])
            i = j
        else:
            i += 1
    merged = []
    for r in runs:                                    # bridge gaps shorter than 150 ms
        if merged and r[0] - merged[-1][1] < 0.15:
            merged[-1][1] = r[1]
        else:
            merged.append(r)
    return [(a, b) for a, b in merged if b - a >= 0.06]


def sentence_cuts(gaps: list[tuple[float, float]], targets: list[float], span: float) -> list[float]:
    """Pick one pause per sentence boundary, in order: long pauses near where the text says the
    boundary should be (a small dynamic programme). Falls back to the text estimate if pauses run out."""
    m, n = len(targets), len(gaps)
    if m == 0:
        return []
    if n < m:
        return targets
    cost = lambda j, i: 4.0 * abs(gaps[i][0] - targets[j]) / span - min(gaps[i][1], 1.5)   # noqa: E731
    INF = float("inf")
    best = [[INF] * n for _ in range(m)]
    back = [[-1] * n for _ in range(m)]
    for i in range(n):
        best[0][i] = cost(0, i)
    for j in range(1, m):
        run, arg = INF, -1
        for i in range(n):
            if i > 0 and best[j - 1][i - 1] < run:
                run, arg = best[j - 1][i - 1], i - 1
            if arg >= 0:
                best[j][i] = run + cost(j, i)
                back[j][i] = arg
    i = min(range(n), key=lambda k: best[m - 1][k])
    out = []
    for j in range(m - 1, -1, -1):
        out.append(gaps[i][0])
        i = back[j][i]
    return out[::-1]


def find_recording(scene_id: str) -> Path | None:
    """voice/scene01.m4a, but also Scene1.m4a, scene1.m4a.mp4 and friends (Windows' default names)."""
    pat = re.compile(rf"^scene0*{int(scene_id)}(\.|$)", re.IGNORECASE)
    hits = [p for p in sorted((HERE / "voice").glob("*"))
            if p.is_file() and pat.match(p.name) and p.suffix.lower() in AUDIO_EXT]
    return hits[0] if hits else None


def squeeze_pauses(pcm: np.ndarray, runs: list[tuple[float, float]], longer: float = 0.22,
                   keep: float = 0.18) -> np.ndarray:
    """Shorten pauses longer than `longer` s to `keep` s (runs are relative to pcm), with short fades."""
    fade = int(0.03 * SR)
    pieces, cursor = [], 0
    for a, b in zip(runs, runs[1:]):
        if b[0] - a[1] > longer:
            cut_from, cut_to = int((a[1] + keep / 2) * SR), int((b[0] - keep / 2) * SR)
            if cut_to > cut_from > cursor:
                pieces.append(pcm[cursor:cut_from].astype(np.float32))
                cursor = cut_to
    pieces.append(pcm[cursor:].astype(np.float32))
    ramp = np.linspace(0.0, 1.0, fade, dtype=np.float32)
    for i, p in enumerate(pieces):
        n = min(fade, len(p) // 2)
        if i > 0:
            p[:n] *= ramp[:n]
        if i < len(pieces) - 1:
            p[len(p) - n:] *= ramp[:n][::-1]
    return np.concatenate(pieces).astype(np.int16)


def speed_up(pcm: np.ndarray, tempo: float) -> np.ndarray:
    """Pitch-preserving tempo change (ffmpeg atempo) on raw PCM."""
    if abs(tempo - 1.0) < 1e-3:
        return pcm
    cmd = [ffmpeg(), "-v", "error", "-f", "s16le", "-ar", str(SR), "-ac", "1", "-i", "-",
           "-af", f"atempo={tempo:.4f}", "-f", "s16le", "-ar", str(SR), "-ac", "1", "-"]
    return np.frombuffer(subprocess.run(cmd, input=pcm.tobytes(), check=True, capture_output=True).stdout,
                         dtype=np.int16)


def my_voice_sentences(path: Path, sc: Scene, tempo: float = 1.0) -> list[tuple[np.ndarray, list[float]]]:
    """Split a scene recording into its sentences: clean-up, long pauses shortened, then optional tempo."""
    pcm = decode(path, "highpass=f=80,afftdn=nf=-25,loudnorm=I=-16:TP=-1.5:LRA=11")
    runs = voiced_runs(pcm)
    if not runs:
        raise SystemExit(f"{path.name}: no speech found")
    s0, s1 = runs[0][0], runs[-1][1]
    gaps = [((a[1] + b[0]) / 2, b[0] - a[1]) for a, b in zip(runs, runs[1:]) if b[0] - a[1] >= 0.2]
    texts = [s.say for s in sc.sentences]
    chars = np.cumsum([len(t) for t in texts])
    targets = [s0 + (s1 - s0) * c / chars[-1] for c in chars[:-1]]
    bounds = [s0] + sentence_cuts(gaps, targets, s1 - s0) + [s1]
    out = []
    for j, txt in enumerate(texts):
        a, b = bounds[j], bounds[j + 1]
        inner = [r for r in runs if r[1] > a and r[0] < b]
        va = max(a, inner[0][0]) if inner else a
        vb = min(b, inner[-1][1]) if inner else b
        t0 = max(0.0, va - 0.05)
        seg = pcm[int(t0 * SR):int((vb + 0.08) * SR)]
        # pauses inside the sentence, measured on the whole clip (a sentence alone is too short to judge)
        seg = squeeze_pauses(seg, [(max(r[0], va) - t0, min(r[1], vb) - t0) for r in inner])
        seg = speed_up(seg, tempo)
        out.append((seg, proportional(txt.split(), 0.05 / tempo, (len(seg) / SR) - 0.08 / tempo)))
    return out


def silent_sentences(sc: Scene) -> list[tuple[np.ndarray, list[float]]]:
    out = []
    for s in sc.sentences:
        words = s.say.split()
        dur = max(1.0, len(words) * 60 / 140)
        out.append((np.zeros(int(dur * SR), dtype=np.int16), proportional(words, 0.0, dur)))
    return out


# ---- timeline -----------------------------------------------------------------------------------
def time_scene(sc: Scene, sents: list[tuple[np.ndarray, list[float]]]) -> dict:
    parts, word_t = [], []
    t = SCENE_LEAD.get(sc.id, LEAD)
    parts.append(np.zeros(int(t * SR), dtype=np.int16))
    for i, (s, (pcm, wt)) in enumerate(zip(sc.sentences, sents)):
        word_t += [t + x for x in wt]
        parts.append(pcm)
        t += len(pcm) / SR
        extra = s.pause_after + (GAP if i < len(sc.sentences) - 1 else SCENE_TAIL.get(sc.id, TAIL))
        parts.append(np.zeros(int(extra * SR), dtype=np.int16))
        t += extra
    pcm = np.concatenate(parts)
    nframes = int(round(len(pcm) / SPF))
    pcm = np.pad(pcm, (0, max(0, nframes * SPF - len(pcm))))[: nframes * SPF]
    dur = nframes / FPS
    say_words = sc.say.split()
    if len(word_t) != len(say_words):
        raise SystemExit(f"scene {sc.id}: {len(word_t)} word times for {len(say_words)} words")
    words = [[round(word_t[i], 3), norm(w)] for i, w in enumerate(say_words)]
    # caption cues start at their first word
    cues, wi = [], 0
    for c in sc.cues:
        cues.append({"start": max(0.0, word_t[wi] - 0.08), "html": c.html, "show": c.show})
        wi += len(c.say.split())
    for a, b in zip(cues, cues[1:]):
        a["end"] = b["start"] - 0.02
    cues[-1]["end"] = dur - 0.12
    # keyword times: the spoken word where each highlighted phrase starts
    kw: dict[str, float] = {}
    wi = 0
    for c in sc.cues:
        prefix = ""
        for s in c.segs:
            if s.kw:
                p = prefix.lstrip()
                idx = len(p.split())
                if p and not p[-1].isspace() and not s.say[:1].isspace():
                    idx -= 1
                name = s.kw.strip()
                key = name if name not in kw else f"{name}#2"
                kw[key] = round(word_t[wi + max(0, idx)], 3)
            prefix += s.say
        wi += len(c.say.split())
    return {"id": sc.id, "title": sc.title, "kind": sc.kind, "tag": sc.tag, "dur": dur, "frames": nframes,
            "pcm": pcm, "cues": cues, "words": words, "kw": kw}


# ---- script, captions ---------------------------------------------------------------------------
def fmt_ts(t: float, sep: str = ",") -> str:
    ms = int(round(t * 1000))
    return f"{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d}{sep}{ms % 1000:03d}"


def write_srt(timeline: list[dict]) -> None:
    lines, n, t0 = [], 1, 0.0
    for T in timeline:
        for c in T["cues"]:
            lines += [str(n), f"{fmt_ts(t0 + c['start'])} --> {fmt_ts(t0 + c['end'])}", c["show"], ""]
            n += 1
        t0 += T["dur"]
    (HERE / "captions.srt").write_text("\n".join(lines), encoding="utf-8")


def write_script(scenes: list[Scene], timeline: list[dict], n: dict, voice: str) -> None:
    total = sum(T["dur"] for T in timeline)
    nwords = sum(len(s.say.split()) for s in scenes)
    out = [
        "# SANCHAR demo video: narration script",
        "",
        f"About {total / 60:.0f} minutes ({int(total // 60)}:{int(total % 60):02d}), {nwords} spoken words, "
        f"{len(scenes)} scenes. Generated by `docs/video/build_video.py` from `docs/video/scenes.py`; "
        "every number is read from `bench/results/`.",
        "",
        "## How to record your own voice",
        "",
        "1. Record one clip per scene and save it as `docs/video/voice/scene01.m4a` … `scene10.m4a`. "
        "Windows Sound Recorder works; `.mp3`, `.wav` and `.ogg` are also fine.",
        "2. Leave about half a second of silence at the start and end of each clip, and a short breath between sentences. "
        "Retake a scene whenever you like; only that clip changes.",
        "3. Where a scene says **(pause)**, pause for about two seconds. The video fast-forwards during it.",
        "4. Run `python docs/video/build_video.py --voice mine --team-name \"…\" --team-id \"…\"`. "
        "Each scene stretches to your pace and the captions follow your sentences. "
        "Any scene you have not recorded keeps the AI voice.",
        "",
        "Keywords are in **bold**. They are highlighted in the captions, so say them clearly.",
        "",
    ]
    t0 = 0.0
    for pos, (sc, T) in enumerate(zip(scenes, timeline), 1):
        out += [f"## Scene {pos}: {sc.title}  ({fmt_ts(t0, '.')[3:8]} to {fmt_ts(t0 + T['dur'], '.')[3:8]}, "
                f"{len(sc.say.split())} words; recording scene{sc.id}.m4a)", ""]
        if sc.tag:
            out += [f"*{sc.tag}*", ""]
        for s in sc.sentences:
            md = lambda x: x.replace("*", "\\*")                          # noqa: E731
            show = " ".join("".join(f"**{md(g.show.strip())}**" if g.kw and g.show.strip() else md(g.show)
                                    for g in c.segs).strip() for c in s.cues)
            show = re.sub(r"\*\*(\s*)\*\*", r"\1", show)
            out.append(f"> {show}" + (" **(pause)**" if s.pause_after >= 1.5 else ""))
            out.append(">")
        out[-1] = ""
        t0 += T["dur"]
    out += [
        "## Pronunciation guide",
        "",
        "| Written | Say |",
        "|---|---|",
        "| SANCHAR | *Sun-chaar* (Hindi संचार: movement, communication) |",
        "| SIH26123 | S-I-H two six one two three |",
        "| A* | A-star |",
        "| Guided-PIBT | Guided P-I-B-T (Priority Inheritance with BackTracking) |",
        "| HMAC-SHA256 | H-mac SHA two fifty-six |",
        "| CRDT | C-R-D-T (conflict-free replicated data type) |",
        "| UDP | U-D-P |",
        "",
        "## What the technical terms mean (for questions)",
        "",
        "- **Discrete-time kinematic model**: robots move at 1 m/s on a roadmap of 1 m cells, one cell per 1 s tick. "
        "Planning and the safety rule reason about which cell each robot occupies at each tick.",
        "- **Windowed space-time A\\***: each robot searches over (cell, time) pairs a few seconds ahead, "
        "treating higher-priority robots' broadcast plans as moving obstacles.",
        "- **Guided-PIBT**: every tick, each robot picks its next cell in priority order, guided by its A\\* path. "
        "A robot in the way inherits the pusher's priority and moves aside (priority inheritance with backtracking). "
        "This is the algorithm family that won the 2023 League of Robot Runners, made fully decentralized here.",
        "- **Claim round and move round**: a robot first broadcasts which cell it wants, then moves only if the safety rule holds.",
        "- **Reach set**: every cell a robot we have not heard from recently could have reached; we never enter it.",
        "- **CRDT task board**: order records merge the same way on every robot in any order, so two robots never both keep one order.",
        "",
        "## If a judge asks about kinematics",
        "",
        "Say it plainly: the coordinator uses a discrete-time kinematic model (1 m/s, one roadmap node per tick). "
        "A continuous execution layer with velocity and acceleration limits for differential-drive robots and "
        "circular footprints sits underneath without changing the coordinator. It is the first Grand Finale item, "
        "and each robot's own safety scanner remains the physical backstop, as on every commercial AMR.",
        "",
        "## Where the numbers come from",
        "",
        f"- `bench/results/summary.json`: {n['families']} scenarios × {n['seeds']} seeds = {n['runs']} SANCHAR runs, "
        f"{n['collisions']} collisions; 6 AMRs {n['r6_exact']}% ± {n['r6_ci']}% faster; 12 AMRs {n['r12']}%; "
        f"live orders {n['rmarket']}%; weak case {n['weak']}% slower; intersection baseline gridlocked "
        f"{n['cross_gridlocked']}/{n['cross_seeds']}; decision p99 up to {n['p99_ms']} ms; "
        f"{n['kb_lo']}–{n['kb_hi']} KB per robot per second.",
        f"- `bench/results/runner_kill.json` and a live run of `sanchar.sim.runner` during the build: "
        f"{n['kill']['orders_done']}/{n['kill']['orders']} orders, {n['kill']['collisions']} collisions, "
        f"robot R{n['kill']['killed'][0]} killed.",
        f"- Voice used for this build: {voice}.",
        "",
    ]
    (HERE / "SCRIPT.md").write_text("\n".join(out), encoding="utf-8")


# ---- slide data ---------------------------------------------------------------------------------
def slide_data(n: dict, team_line: str) -> dict:
    S = load_summary()
    rows = {f: {"red": red(s), "ci": ci(s), "gridlocked": s["baseline_gridlocked_seeds"], "seeds": s["seeds"]}
            for f, s in S.items()}
    fmt = {k: (f"{v}" if not isinstance(v, float) else f"{v:g}") for k, v in n.items() if k != "kill"}
    fmt["r6_exact"] = f"{n['r6_exact']:.1f}"
    fmt["p99_ms"] = f"{n['p99_ms']:.1f}"
    return {"rows": rows, "term": terminal_lines(n), "n": fmt, "team_line": team_line,
            "team_sep": "· " if team_line else ""}


def terminal_lines(n: dict) -> list[list[str]]:
    K = n["kill"]
    kill_at = 40
    cmd = ["-m", "sanchar.sim.runner", "--robots", str(K["robots"]), "--orders", str(K["orders"]),
           "--seed", str(K["seed"]), "--kill", f"{K['killed'][0]}@{kill_at}"]
    lines = [["cmd", "$ python " + " ".join(cmd)]]
    try:
        res = subprocess.run([sys.executable, *cmd], cwd=ROOT, capture_output=True, text=True, timeout=180)
        raw = res.stdout.splitlines()
        result = json.loads(next(x for x in raw if x.startswith("RESULT"))[7:])
    except Exception as exc:                         # fall back to the committed result
        log(f"  runner failed ({exc!r}); using bench/results/runner_kill.json")
        raw, result = [], K
    keep_t = {0, 60, 100, 140, 200}
    for x in raw:
        m = re.match(r"\[t=\s*(\d+)\]", x)
        if x.startswith("started"):
            lines.append(["", x])
        elif "killed robot process" in x or "stopped responding" in x:
            lines.append(["kill", x])
        elif "technician" in x:
            lines.append(["", x])
        elif m and "alive" in x and int(m.group(1)) in keep_t:
            lines.append(["", x])
    lines.append(["ok", f"RESULT  orders {result['orders_done']}/{result['orders']} delivered · "
                        f"collisions {result['collisions']} · robots alive {result['robots_alive']}/{result['robots']}"])
    if result["orders_done"] != K["orders_done"] or result["collisions"] != 0:
        log("  WARNING: the live runner result differs from bench/results/runner_kill.json")
    return lines


# ---- rendering ----------------------------------------------------------------------------------
class Encoder:
    def __init__(self, path: Path, frames: int, fade_in: bool, fade_out: bool):
        vf = []
        if fade_in:
            vf.append("fade=t=in:s=0:n=8")
        if fade_out:
            vf.append(f"fade=t=out:s={max(0, frames - 8)}:n=8")
        vf.append("format=yuv420p")
        self.path, self.frames, self.n = path, frames, 0
        # low priority and few threads: the live dashboard measures its robots' decision time while
        # being filmed, and a busy encoder would inflate that number on screen
        self.proc = subprocess.Popen(
            [ffmpeg(), "-y", "-v", "error", "-f", "image2pipe", "-framerate", str(FPS), "-c:v", "mjpeg", "-i", "-",
             "-vf", ",".join(vf), "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-r", str(FPS),
             "-g", "60", "-threads", "2", str(path)], stdin=subprocess.PIPE,
            creationflags=getattr(subprocess, "IDLE_PRIORITY_CLASS", 0))
        self.last: bytes | None = None

    def write(self, jpg: bytes) -> None:
        if self.n < self.frames:
            self.proc.stdin.write(jpg)
            self.n += 1
            self.last = jpg

    def close(self) -> None:
        while self.n < self.frames and self.last is not None:     # pad (never expected)
            self.write(self.last)
        self.proc.stdin.close()
        if self.proc.wait() != 0:
            raise SystemExit(f"ffmpeg failed on {self.path.name}")


def launch(pw):
    for ch in ("msedge", "chrome"):
        try:
            return pw.chromium.launch(channel=ch, headless=True)
        except Exception as exc:
            log(f"  {ch} not usable ({exc.__class__.__name__}); trying the next browser")
    return pw.chromium.launch(headless=True)


def render_slide(browser, T: dict, data: dict, enc: Encoder) -> None:
    page = browser.new_page(viewport={"width": 1280, "height": 720}, device_scale_factor=1.5, color_scheme="dark")
    warns: list[str] = []
    page.on("console", lambda m: warns.append(m.text) if m.type in ("warning", "error") else None)
    page.on("pageerror", lambda e: warns.append(str(e)))
    page.goto((HERE / "slides" / f"scene{T['id']}.html").as_uri())
    page.evaluate("document.fonts.ready.then(() => true)")
    page.evaluate("(d) => V.init(d)", {"dur": T["dur"], "cues": T["cues"], "words": T["words"], "kw": T["kw"],
                                       "tag": T["tag"], "data": data, "n": data["n"],
                                       "team_line": data["team_line"], "team_sep": data["team_sep"]})
    last_sig, jpg = None, None
    for f in range(T["frames"]):
        sig = page.evaluate("(t) => V.setFrame(t)", f / FPS)
        if sig != last_sig or jpg is None:
            jpg = page.screenshot(type="jpeg", quality=92)
            last_sig = sig
        enc.write(jpg)
    if warns:
        raise SystemExit(f"scene {T['id']} slide problems: {sorted(set(warns))}")
    page.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--team-name", default="")
    ap.add_argument("--team-id", default="")
    ap.add_argument("--voice", choices=["tts", "mine", "none"], default="tts")
    ap.add_argument("--tts-voice", default="en-IN-NeerjaNeural", help="en-IN-PrabhatNeural for a male voice")
    ap.add_argument("--tts-rate", default="+20%")
    ap.add_argument("--max-length", type=float, default=None,
                    help="with --voice mine: speed your recordings up (pitch unchanged) so the video fits in this many seconds")
    ap.add_argument("--only-script", action="store_true", help="write SCRIPT.md and captions.srt, skip the video")
    ap.add_argument("--check", action="store_true", help="also export a frame at every cue and a contact sheet")
    a = ap.parse_args(argv)
    t_start = time.time()

    n = numbers()
    scenes = build_scenes(n)
    check_numbers(scenes, n)
    team_line = " · ".join(x for x in (a.team_name, f"Team ID {a.team_id}" if a.team_id else "") if x)

    def narrate(tempo: float) -> tuple[list[dict], list[str], float]:
        """Timeline for every scene, the audio source of each, and seconds of recorded (speed-able) speech."""
        timeline, sources, recorded = [], [], 0.0
        for sc in scenes:
            mine = find_recording(sc.id) if a.voice == "mine" else None
            if mine is not None:
                sents, src = my_voice_sentences(mine, sc, tempo), mine.name
                recorded += sum(len(p) for p, _ in sents) / SR
            elif a.voice == "none":
                sents, src = silent_sentences(sc), "silent"
            else:
                if a.voice == "mine":
                    log(f"  scene {sc.id}: no recording in docs/video/voice/, using the AI voice")
                sents, src = [tts_sentence(s.say, a.tts_voice, a.tts_rate) for s in sc.sentences], a.tts_voice
            timeline.append(time_scene(sc, sents))
            sources.append(src)
        return timeline, sources, recorded

    log("1/5 narration audio")
    tempo = 1.0
    timeline, sources, recorded = narrate(tempo)
    total = sum(T["dur"] for T in timeline)
    if a.max_length and a.voice == "mine" and recorded and total > a.max_length:
        for _ in range(4):                            # fixed point: kept pauses do not scale with tempo
            other = total - recorded
            tempo = min(MAX_TEMPO, tempo * recorded / max(1.0, a.max_length - 0.4 - other))
            timeline, sources, recorded = narrate(tempo)
            total = sum(T["dur"] for T in timeline)
            if total <= a.max_length or tempo >= MAX_TEMPO:
                break
        log(f"  your recordings play at {tempo:.2f}x (pitch unchanged) to fit {a.max_length:.0f} s")
        if total > a.max_length:
            log(f"  WARNING: still {total:.1f} s at the {MAX_TEMPO}x limit")
    for T, src in zip(timeline, sources):
        log(f"  scene {T['id']} {T['title']:32s} {T['dur']:5.1f} s  ({src})")
    log(f"  total {int(total // 60)}:{total % 60:04.1f}")
    write_srt(timeline)
    voice_desc = a.tts_voice if a.voice == "tts" else a.voice
    if a.voice == "mine":
        voice_desc = "your recordings in docs/video/voice/, long pauses shortened" + (
            f", played at {tempo:.2f}x (pitch unchanged)" if tempo > 1.001 else "")
    write_script(scenes, timeline, n, voice_desc)
    if a.only_script:
        return

    log("2/5 planning the live dashboard scenes")
    from director import Director, Filmer
    dash_T = [T for T in timeline if T["kind"] == "dash"]
    t0, dash_scenes, dash_cues = 0.0, [], []
    for T in dash_T:
        dash_scenes.append({"id": T["id"], "start": t0, "dur": T["dur"], "kw": T["kw"], "tag": T["tag"]})
        dash_cues += [{**c, "start": c["start"] + t0, "end": c["end"] + t0} for c in T["cues"]]
        t0 += T["dur"]
    dash = {"dur": t0, "scenes": dash_scenes, "cues": dash_cues}
    director = Director(dash, log=log, bench_p99_ms=n["p99_ms"])
    hint_file = BUILD / "plan_hint.json"
    hints = json.loads(hint_file.read_text(encoding="utf-8")) if hint_file.exists() else {}
    dash_key = hashlib.sha1(json.dumps(dash, sort_keys=True).encode()).hexdigest()
    plan = director.plan(first=tuple(hints[dash_key]) if dash_key in hints else None)
    hints[dash_key] = [plan.seed, plan.cand]
    hint_file.write_text(json.dumps(hints), encoding="utf-8")

    log("3/5 rendering slides and filming the dashboard")
    data = slide_data(n, team_line)
    clips = BUILD / "clips"
    clips.mkdir(parents=True, exist_ok=True)
    css = (HERE / "slides" / "video.css").read_text(encoding="utf-8")
    js = (HERE / "slides" / "video.js").read_text(encoding="utf-8")
    from playwright.sync_api import sync_playwright
    with sync_playwright() as pw:
        browser = launch(pw)
        for T in timeline:
            if T["kind"] != "slide":
                continue
            ts = time.time()
            clip, keyf = clips / f"scene{T['id']}.mp4", clips / f"scene{T['id']}.key"
            used = data if T["id"] == "10" else {k: v for k, v in data.items() if k != "term"}   # term: live PIDs
            key = hashlib.sha1(json.dumps([{k: v for k, v in T.items() if k != "pcm"}, used], sort_keys=True).encode()
                               + (HERE / "slides" / f"scene{T['id']}.html").read_bytes() + css.encode() + js.encode()).hexdigest()
            if clip.exists() and keyf.exists() and keyf.read_text() == key:
                log(f"  scene {T['id']} slide  unchanged, reused")
                continue
            enc = Encoder(clip, T["frames"], True, True)
            render_slide(browser, T, data, enc)
            enc.close()
            keyf.write_text(key)
            log(f"  scene {T['id']} slide  {T['frames']} frames  {time.time() - ts:.0f} s")
        ts = time.time()
        encs = {T["id"]: Encoder(clips / f"scene{T['id']}.mp4", T["frames"], i == 0, i == len(dash_T) - 1)
                for i, T in enumerate(dash_T)}
        Filmer(director, plan, browser, css, js, lambda sid, jpg: encs[sid].write(jpg)).film()
        for e in encs.values():
            e.close()
        log(f"  scenes {'-'.join(T['id'] for T in dash_T)} dashboard, seed {plan.seed}  {time.time() - ts:.0f} s")
        browser.close()

    log("4/5 audio and final encode")
    audio = BUILD / "narration.wav"
    import wave
    with wave.open(str(audio), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(np.concatenate([T["pcm"] for T in timeline]).tobytes())
    lst = BUILD / "clips.txt"
    lst.write_text("".join(f"file '{(clips / f'scene{T['id']}.mp4').as_posix()}'\n" for T in timeline), encoding="utf-8")
    video = BUILD / "video_only.mp4"
    subprocess.run([ffmpeg(), "-y", "-v", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(video)], check=True)
    af = "loudnorm=I=-16:TP=-1.5:LRA=11" if a.voice != "none" else "anull"
    subprocess.run([ffmpeg(), "-y", "-v", "error", "-i", str(video), "-i", str(audio), "-map", "0:v", "-map", "1:a",
                    "-c:v", "copy", "-af", af, "-ar", str(SR), "-c:a", "aac", "-b:a", "192k", "-shortest",
                    "-movflags", "+faststart", str(OUT_MP4)], check=True)

    log("5/5 checks")
    info = subprocess.run([ffmpeg(), "-hide_banner", "-i", str(OUT_MP4)], capture_output=True, text=True).stderr
    dur = re.search(r"Duration: (\d+):(\d+):([\d.]+)", info)
    secs = int(dur.group(1)) * 3600 + int(dur.group(2)) * 60 + float(dur.group(3))
    vs = re.search(r"Video: .*?(\d{3,4})x(\d{3,4}).*?(\d+(?:\.\d+)?) fps", info)
    has_audio = "Audio:" in info
    log(f"  {OUT_MP4.name}: {int(secs // 60)}:{secs % 60:04.1f}, {vs.group(1)}x{vs.group(2)} @ {vs.group(3)} fps, "
        f"audio {'yes' if has_audio else 'NO'}, {OUT_MP4.stat().st_size / 1e6:.1f} MB")
    if abs(secs - total) > 0.5 or (vs.group(1), vs.group(2)) != ("1920", "1080") or not has_audio:
        raise SystemExit("output check failed")
    if a.check:
        chk = BUILD / "check"
        chk.mkdir(exist_ok=True)
        for f in chk.glob("*.jpg"):
            f.unlink()
        t0 = 0.0
        for T in timeline:
            for i, c in enumerate(T["cues"]):
                ts = t0 + min(c["end"] - 0.05, c["start"] + 0.9)
                subprocess.run([ffmpeg(), "-y", "-v", "error", "-ss", f"{ts:.2f}", "-i", str(OUT_MP4), "-frames:v", "1",
                                "-q:v", "3", str(chk / f"s{T['id']}_c{i:02d}.jpg")], check=True)
            t0 += T["dur"]
        subprocess.run([ffmpeg(), "-y", "-v", "error", "-i", str(OUT_MP4), "-vf", "fps=1/5,scale=480:-1,tile=4x10",
                        "-frames:v", "1", str(chk / "contact_sheet.jpg")], check=True)
        log(f"  frames for review in {chk.relative_to(ROOT)}")
    log(f"done in {time.time() - t_start:.0f} s -> {OUT_MP4.relative_to(ROOT)}")


if __name__ == "__main__":
    main()

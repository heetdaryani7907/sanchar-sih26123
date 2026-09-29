// SANCHAR demo video engine. Deterministic and time-driven: the builder calls V.setFrame(t) for
// every frame and screenshots the page. Slides declare reveals with data-at / data-until:
//   kw:<keyword>   when that keyword is spoken      w:<word>[#n]  when the n-th such word is spoken
//   cue:<i>        start of caption cue i            t:<sec>, end:<-sec>
// with an optional "@+0.3" offset. The same file draws captions and overlays on the dashboard.
(function () {
  const V = (window.V = {});
  let D = null, anims = [], overlays = [], lastCue = -2, layer = null, cap = null, cursor = null;
  const clamp = (x) => Math.max(0, Math.min(1, x));
  const ease = (x) => (x < 0.5 ? 2 * x * x : 1 - Math.pow(-2 * x + 2, 2) / 2);
  const get = (o, path) => path.split(".").reduce((a, k) => (a == null ? a : a[k]), o);

  V.init = function (data) {
    D = data;
    document.documentElement.style.setProperty("--ov", String(1.5 / (window.devicePixelRatio || 1.5)));
    document.querySelectorAll("[data-bind]").forEach((el) => {
      const v = get(D, el.dataset.bind);
      if (v !== undefined && v !== null) el.textContent = v;
    });
    if (window.onInit) window.onInit(D);
    anims = [...document.querySelectorAll("[data-at]")].map((el) => ({
      el, at: V.when(el.dataset.at), until: el.dataset.until ? V.when(el.dataset.until) : null,
      dur: parseFloat(el.dataset.dur || "0.45"),
    }));
    layer = document.getElementById("ov-layer") || Object.assign(document.createElement("div"), { id: "ov-layer" });
    cap = document.getElementById("ov-cap") || Object.assign(document.createElement("div"), { id: "ov-cap" });
    document.body.append(layer, cap);
    overlays = (D.overlays || []).map((o) => ({ ...o, el: null }));
    if (D.cursor) {
      cursor = document.createElement("div");
      cursor.id = "ov-cursor";
      cursor.innerHTML = '<div class="click"></div><svg viewBox="0 0 24 24" width="100%" height="100%">' +
        '<path d="M3 2 L3 20 L8 15 L11.5 22.5 L14.5 21 L11 14 L18 14 Z" fill="#fff" stroke="#111" stroke-width="1.4" stroke-linejoin="round"/></svg>';
      document.body.append(cursor);
    }
    return true;
  };

  V.when = function (spec) {
    let off = 0;
    const m = spec.match(/^(.*)@([+-]?\d*\.?\d+)$/);
    if (m) { spec = m[1]; off = parseFloat(m[2]); }
    const i = spec.indexOf(":"), kind = spec.slice(0, i), arg = spec.slice(i + 1);
    let t = null;
    if (kind === "kw") t = D.kw[arg];
    else if (kind === "w") {
      const [w, nth] = arg.split("#");
      let c = 0;
      for (const [tt, ww] of D.words) if (ww === w.toLowerCase() && ++c === (+nth || 1)) { t = tt; break; }
    } else if (kind === "cue") t = D.cues[+arg] && D.cues[+arg].start;
    else if (kind === "t") t = +arg;
    else if (kind === "end") t = D.dur + +arg;
    if (t === null || t === undefined) { console.warn("unresolved timing " + spec); t = 1e9; }
    return t + off;
  };

  // ---- overlay geometry (dashboard) ------------------------------------------------------------
  function mapRect() { const c = document.getElementById("map"); return c ? c.getBoundingClientRect() : { left: 0, top: 0 }; }
  function nodePt(n) { const r = mapRect(), p = window.nodeXY(n); return [r.left + p[0], r.top + p[1]]; }
  function robotPt(id) {
    const a = (window.__prevPos || {})[id], b = (window.__curPos || {})[id];
    const r = mapRect(), pa = window.nodeXY(a ?? b), pb = window.nodeXY(b ?? a);
    const al = window.__alpha ? window.__alpha() : 1;
    return [r.left + pa[0] + (pb[0] - pa[0]) * al, r.top + pa[1] + (pb[1] - pa[1]) * al];
  }
  function anchor(o) {
    if (o.robot !== undefined) return robotPt(o.robot);
    if (o.node !== undefined) return nodePt(o.node);
    if (o.xy) return o.xy;
    if (o.sel) { const e = document.querySelector(o.sel); if (!e) return [0, 0]; const b = e.getBoundingClientRect(); return [b.left + b.width / 2, b.top + b.height / 2]; }
    return [0, 0];
  }
  function rectOf(o) {
    if (o.nodes) {
      const pts = o.nodes.map(nodePt), c = window.__cell ? window.__cell() : 30;
      const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
      return [Math.min(...xs) - c / 2, Math.min(...ys) - c / 2, Math.max(...xs) - Math.min(...xs) + c, Math.max(...ys) - Math.min(...ys) + c];
    }
    if (o.sel) { const e = document.querySelector(o.sel); if (!e) return [0, 0, 0, 0]; const b = e.getBoundingClientRect(); return [b.left, b.top, b.width, b.height]; }
    return o.rect;
  }
  function place(el, x, y, w, h) {
    el.style.left = x + "px"; el.style.top = y + "px";
    if (w !== undefined) { el.style.width = w + "px"; el.style.height = h + "px"; }
  }
  const OV = () => parseFloat(getComputedStyle(document.documentElement).getPropertyValue("--ov")) || 1;

  function drawOverlay(o, p, t) {
    const s = OV();
    if (!o.el) {
      o.el = document.createElement("div");
      o.el.className = { spot: "ov-spot", box: "ov-box", ring: "ov-ring", badge: "ov-badge", tag: "ov-tag", callout: "ov-label", note: "ov-note" }[o.type] || "";
      if (o.type === "badge" || o.type === "tag" || o.type === "note") o.el.textContent = o.text;
      if (o.type === "callout") {
        o.line = document.createElement("div"); o.line.className = "ov-line"; layer.append(o.line);
        o.ring = document.createElement("div"); o.ring.className = "ov-ring"; layer.append(o.ring);
        o.el.textContent = o.text;
      }
      if (o.type === "box" && o.text) {
        o.lab = document.createElement("div"); o.lab.className = "ov-label"; o.lab.textContent = o.text; layer.append(o.lab);
      }
      layer.append(o.el);
    }
    o.el.style.setProperty("--p", p);
    o.el.style.display = p > 0 ? "block" : "none";
    if (o.type === "spot" || o.type === "box") {
      const pad = (o.pad ?? 6) * s, [x, y, w, h] = rectOf(o);
      place(o.el, x - pad, y - pad, w + 2 * pad, h + 2 * pad);
      if (o.lab) {
        o.lab.style.setProperty("--p", p); o.lab.style.display = p > 0 ? "block" : "none";
        const lw = o.lab.offsetWidth, lh = o.lab.offsetHeight;
        const below = o.labelPos === "below";
        place(o.lab, Math.max(8, x + w / 2 - lw / 2), below ? y + h + pad + 8 * s : y - pad - lh - 8 * s);
      }
    } else if (o.type === "ring") {
      const [x, y] = anchor(o), r = (o.r || 26) * s;
      place(o.el, x - r, y - r, 2 * r, 2 * r);
    } else if (o.type === "callout") {
      const [x, y] = anchor(o), r = (o.r || 22) * s;
      const dx = (o.dx ?? 90) * s, dy = (o.dy ?? -70) * s;
      const lw = o.el.offsetWidth, lh = o.el.offsetHeight;
      let lx = x + dx - (dx < 0 ? lw : 0), ly = y + dy - lh / 2;
      lx = Math.max(8, Math.min(window.innerWidth - lw - 8, lx));
      place(o.el, lx, ly);
      o.ring.style.setProperty("--p", p); o.ring.style.display = p > 0 ? "block" : "none";
      place(o.ring, x - r, y - r, 2 * r, 2 * r);
      const ex = dx < 0 ? lx + lw : lx, ey = ly + lh / 2;
      const ang = Math.atan2(ey - y, ex - x), len = Math.max(0, Math.hypot(ex - x, ey - y) - r);
      o.line.style.setProperty("--p", p); o.line.style.display = p > 0 ? "block" : "none";
      place(o.line, x + Math.cos(ang) * r, y + Math.sin(ang) * r);
      o.line.style.width = len + "px"; o.line.style.transform = `rotate(${ang}rad)`;
    } else if (o.type === "chips") {
      // a row of keyword chips under the map legend; each chip fades in at its own time
      if (!o.kids) {
        o.el.className = "ov-chips";
        o.kids = o.items.map((it) => { const c = document.createElement("span"); c.textContent = it.text; o.el.append(c); return c; });
      }
      o.el.style.display = p > 0 ? "flex" : "none";
      const lg = document.querySelector(o.below || ".legend");
      if (lg) { const b = lg.getBoundingClientRect(); place(o.el, b.left, b.bottom + 12 * s); o.el.style.maxWidth = (o.width || 930) + "px"; }
      o.items.forEach((it, i) => {
        const q = +(ease(clamp((t - it.at) / 0.35))).toFixed(3);
        o.kids[i].style.opacity = q;
        o.kids[i].style.display = q > 0 ? "inline-block" : "none";
      });
    } else if (o.type === "note") {
      // a quiet footnote to the right of an element, as wide as its container allows (an empty grid cell)
      const [x, y, w, h] = rectOf(o), gap = 10 * s;
      const c = document.querySelector(o.within).getBoundingClientRect();
      o.el.style.display = p > 0 ? "flex" : "none";
      place(o.el, x + w + gap, y, c.right - (x + w + gap), h);
    } else if (o.type === "mark") {
      if (p > 0) document.querySelectorAll(o.sel).forEach((e) => { if (new RegExp(o.re).test(e.textContent)) e.classList.add("ov-mark"); });
      else document.querySelectorAll(".ov-mark").forEach((e) => e.classList.remove("ov-mark"));
    }
  }

  function drawCursor(t) {
    // D.cursor is a list of key-frame sequences; the cursor shows only while a sequence is running
    const K = (D.cursor || []).find((seq) => seq.length && t >= seq[0].t - 0.001 && t <= seq[seq.length - 1].t + 0.6);
    if (!K) { if (cursor) cursor.style.display = "none"; return ""; }
    let i = 0;
    while (i < K.length - 1 && K[i + 1].t <= t) i++;
    const a = K[i], b = K[Math.min(i + 1, K.length - 1)];
    const f = b.t > a.t ? ease(clamp((t - a.t) / (b.t - a.t))) : 1;
    const pa = a.sel ? anchor({ sel: a.sel }) : a.node !== undefined ? nodePt(a.node) : a.xy;
    const pb = b.sel ? anchor({ sel: b.sel }) : b.node !== undefined ? nodePt(b.node) : b.xy;
    const x = pa[0] + (pb[0] - pa[0]) * f, y = pa[1] + (pb[1] - pa[1]) * f;
    cursor.style.display = "block";
    cursor.style.left = x - 3 * OV() + "px"; cursor.style.top = y - 2 * OV() + "px";
    let click = 0;
    for (const k of K) if (k.click && t >= k.t && t < k.t + 0.45) click = 1 - (t - k.t) / 0.45;
    const c = cursor.querySelector(".click");
    c.style.opacity = click; c.style.transform = `scale(${1.6 - click * 0.6})`;
    return x.toFixed(1) + "," + y.toFixed(1) + "," + click.toFixed(2);
  }

  V.setFrame = function (t) {
    const sig = [];
    for (const a of anims) {
      const p = clamp((t - a.at) / a.dur), q = a.until === null ? 0 : clamp((t - a.until) / a.dur);
      const v = +(ease(p) * (1 - ease(q))).toFixed(3);
      a.el.style.setProperty("--p", v);
      a.el.classList.toggle("on", t >= a.at && (a.until === null || t < a.until));
      sig.push(v);
    }
    for (const o of overlays) {
      const d = o.fade ?? 0.35;
      const p = +(ease(clamp((t - o.at) / d)) * (o.until == null ? 1 : 1 - ease(clamp((t - o.until) / d)))).toFixed(3);
      if (p > 0 || o.el) drawOverlay(o, p, t);
      sig.push(p);
    }
    let ci = -1;
    for (let i = 0; i < D.cues.length; i++) if (t >= D.cues[i].start && t < D.cues[i].end) ci = i;
    if (ci !== lastCue) { cap.innerHTML = ci >= 0 ? D.cues[ci].html : ""; lastCue = ci; }
    sig.push(ci);
    sig.push(drawCursor(t));
    if (window.onFrame) sig.push(window.onFrame(t, D));
    return JSON.stringify(sig);
  };
})();

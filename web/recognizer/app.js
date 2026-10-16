import { recognize } from "./recognizer.js";

const REASONS = {
  dtw_distance_too_large: "No template is close enough",
  dtw_margin_too_small: "Two symbols are too close to call",
  dtw_confidence_too_low: "Confidence below the gate",
  dtw_no_candidates: "Nothing to compare",
};

const $ = (id) => document.getElementById(id);
const pad = $("pad");
const padCtx = pad.getContext("2d");
let config = null;
let bank = null;
let strokes = [];
let current = null;
let replayTimer = null;

const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (value, digits = 3) => (Number.isFinite(value) ? value.toFixed(digits) : "∞");
const pct = (value) => `${Math.round(value * 100)}%`;

function setupCanvas(canvas, ctx) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return rect;
}

function drawGrid(ctx, w, h) {
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = cssVar("--grid");
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(w / 2, 0);
  ctx.lineTo(w / 2, h);
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();
}

function drawPolyline(ctx, pts, color, width, startDot = false) {
  if (!pts.length) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.beginPath();
  pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y)));
  ctx.stroke();
  if (startDot) {
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pts[0][0], pts[0][1], width * 1.6, 0, Math.PI * 2);
    ctx.fill();
  }
}

// Normalized y-up points in about [-1, 1] -> canvas pixels (y-down).
function toCanvas(pts, size, margin) {
  const half = size / 2 - margin;
  return pts.map(([x, y]) => [size / 2 + x * half, size / 2 - y * half]);
}

function redrawPad() {
  const rect = pad.getBoundingClientRect();
  drawGrid(padCtx, rect.width, rect.height);
  for (const stroke of strokes) drawPolyline(padCtx, stroke, cssVar("--ink"), 4, true);
  if (current) drawPolyline(padCtx, current, cssVar("--ink"), 4, true);
}

function updateButtons() {
  const empty = strokes.length === 0;
  $("undo").disabled = empty;
  $("clear").disabled = empty;
}

function showEmpty(message) {
  $("status").hidden = false;
  $("status").textContent = message;
  for (const id of ["best", "overlay-fig", "gate", "badge"]) $(id).hidden = true;
  $("cands").replaceChildren();
}

function runRecognition() {
  updateButtons();
  const points = strokes.flat();
  if (!bank) return;
  if (points.length < 2) {
    showEmpty(strokes.length ? "Keep drawing…" : "Draw a symbol to see the ranking.");
    return;
  }
  // Canvas y grows downward; the runtime and the bank use y-up.
  const query = points.map(([x, y]) => [x, -y]);
  const t0 = performance.now();
  const result = recognize(query, bank, {
    topK: config.top_k,
    maxDistance: config.max_distance,
    minMargin: config.min_margin,
    minConfidence: config.min_confidence,
    labels: config.labels,
  });
  renderResult(result, performance.now() - t0, strokes.length);
}

function renderResult({ candidates, gate, ranking, query }, ms, strokeCount) {
  $("status").hidden = true;
  const best = candidates[0];
  const accepted = gate.accepted;
  const badge = $("badge");
  badge.hidden = false;
  badge.className = `badge ${accepted ? "ok" : "warn"}`;
  badge.textContent = accepted ? "Accepted" : "Uncertain";

  const sem = config.semantics[best.label] || {};
  $("best").hidden = false;
  $("best").classList.toggle("uncertain", !accepted);
  $("best-label").textContent = best.label;
  $("best-name").textContent = accepted
    ? `${sem.display_name || best.label} would be offered for confirmation`
    : `Rejected: ${REASONS[gate.uncertainty_reason] || gate.uncertainty_reason}`;
  $("best-desc").textContent = accepted
    ? sem.description || ""
    : `Closest is ${best.label} (${sem.display_name || ""}); the robot side would offer no task.`;

  const items = candidates.map((c) => {
    const li = document.createElement("li");
    li.className = "cand";
    const name = (config.semantics[c.label] || {}).display_name || "";
    li.innerHTML = `<span class="lab"></span><span class="bar"><span></span></span><span class="num"></span><span class="name"></span>`;
    li.querySelector(".lab").textContent = c.label;
    li.querySelector(".bar > span").style.width = pct(c.confidence);
    li.querySelector(".num").textContent = `conf ${pct(c.confidence)} · dist ${fmt(c.distance)}`;
    li.querySelector(".name").textContent = name;
    li.setAttribute("aria-label", `Rank ${c.rank}: ${c.label} ${name}, confidence ${pct(c.confidence)}, distance ${fmt(c.distance)}`);
    return li;
  });
  $("cands").replaceChildren(...items);

  const template = bank.templates.find((t) => t.id === ranking.nearest_template);
  const ov = $("overlay");
  const ovCtx = ov.getContext("2d");
  const rect = setupCanvas(ov, ovCtx);
  drawGrid(ovCtx, rect.width, rect.height);
  if (template) drawPolyline(ovCtx, toCanvas(template.points, rect.width, 12), cssVar("--tmpl"), 2.5, true);
  drawPolyline(ovCtx, toCanvas(query, rect.width, 12), cssVar("--ink"), 2, true);
  $("tmpl-id").textContent = ranking.nearest_template;
  $("overlay-fig").hidden = false;

  const rows = [
    ["Best distance", `${fmt(gate.best_distance)} ≤ ${gate.max_distance}`, gate.best_distance > gate.max_distance],
    ["Margin to next symbol", `${fmt(gate.margin)} ≥ ${gate.min_margin}`, gate.margin < gate.min_margin],
    ["Top-1 confidence", `${fmt(gate.top1_confidence, 2)} ≥ ${gate.min_confidence}`, gate.top1_confidence < gate.min_confidence],
    ["Templates compared", String(ranking.n_templates_compared), false],
    ["Strokes / points", `${strokeCount} / ${strokes.flat().length}`, false],
    ["Time in your browser", `${ms.toFixed(1)} ms`, false],
  ];
  const dl = $("gate");
  dl.replaceChildren();
  for (const [k, v, fail] of rows) {
    const dt = document.createElement("dt");
    dt.textContent = k;
    const dd = document.createElement("dd");
    dd.textContent = v;
    if (fail) dd.className = "fail";
    dl.append(dt, dd);
  }
  dl.hidden = false;
}

function stopReplay() {
  if (replayTimer) cancelAnimationFrame(replayTimer);
  replayTimer = null;
}

function padPoint(event) {
  const rect = pad.getBoundingClientRect();
  return [event.clientX - rect.left, event.clientY - rect.top];
}

pad.addEventListener("pointerdown", (event) => {
  if (!bank) return;
  stopReplay();
  pad.setPointerCapture(event.pointerId);
  current = [padPoint(event)];
  redrawPad();
});
pad.addEventListener("pointermove", (event) => {
  if (!current) return;
  const events = event.getCoalescedEvents ? event.getCoalescedEvents() : [event];
  for (const e of events) current.push(padPoint(e));
  redrawPad();
});
function endStroke() {
  if (!current) return;
  if (current.length > 1) strokes.push(current);
  current = null;
  redrawPad();
  runRecognition();
}
pad.addEventListener("pointerup", endStroke);
pad.addEventListener("pointercancel", endStroke);

$("undo").addEventListener("click", () => {
  stopReplay();
  strokes.pop();
  redrawPad();
  runRecognition();
});
$("clear").addEventListener("click", () => {
  stopReplay();
  strokes = [];
  redrawPad();
  runRecognition();
});

function replayTemplate(template) {
  stopReplay();
  const rect = pad.getBoundingClientRect();
  const target = toCanvas(template.points, rect.width, rect.width * 0.18);
  strokes = [];
  current = [];
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const start = performance.now();
  const duration = reduceMotion ? 0 : 700;
  const step = (now) => {
    const n = duration ? Math.ceil((target.length * Math.min(1, (now - start) / duration))) : target.length;
    current = target.slice(0, Math.max(1, n));
    redrawPad();
    if (n < target.length) {
      replayTimer = requestAnimationFrame(step);
    } else {
      replayTimer = null;
      endStroke();
    }
  };
  replayTimer = requestAnimationFrame(step);
}

function buildGallery() {
  const gallery = $("gallery");
  for (const label of config.labels) {
    const template = bank.templates.find((t) => t.label === label);
    if (!template) continue;
    const name = (config.semantics[label] || {}).display_name || "";
    const tile = document.createElement("button");
    tile.type = "button";
    tile.className = "tile";
    tile.setAttribute("aria-label", `Replay a stored ${label} template (${name})`);
    const canvas = document.createElement("canvas");
    canvas.setAttribute("aria-hidden", "true");
    const labelEl = document.createElement("span");
    labelEl.className = "t-label";
    labelEl.textContent = label;
    const nameEl = document.createElement("span");
    nameEl.className = "t-name";
    nameEl.textContent = name;
    tile.append(canvas, labelEl, nameEl);
    tile.addEventListener("click", () => replayTemplate(template));
    gallery.append(tile);
    const ctx = canvas.getContext("2d");
    const rect = setupCanvas(canvas, ctx);
    drawPolyline(ctx, toCanvas(template.points, rect.width, 10), cssVar("--ink"), 2, true);
  }
}

async function load() {
  try {
    config = await (await fetch("config.json")).json();
    bank = await (await fetch(config.bank_file)).json();
  } catch (err) {
    showEmpty("Could not load the template bank. Serve this folder over HTTP (see the README).");
    console.error(err);
    return;
  }
  bank.templates = bank.templates.filter((t) => config.labels.includes(String(t.label)));
  for (const el of document.querySelectorAll("[data-cfg]")) el.textContent = config[el.dataset.cfg];
  $("bank-name").textContent = config.bank_name;
  $("bank-count").textContent = String(bank.templates.length);
  setupCanvas(pad, padCtx);
  redrawPad();
  buildGallery();
  showEmpty("Draw a symbol to see the ranking.");
}

window.addEventListener("resize", () => {
  if (!bank) return;
  // Strokes are kept in CSS pixels of the old size; scale them to the new one.
  const before = pad.width / (window.devicePixelRatio || 1);
  const rect = setupCanvas(pad, padCtx);
  const k = before ? rect.width / before : 1;
  strokes = strokes.map((s) => s.map(([x, y]) => [x * k, y * k]));
  redrawPad();
});

load();

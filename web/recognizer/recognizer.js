// Browser port of the runtime DTW template-bank recognizer.
//
// Mirrors colmag_ros/src/colmag_ros/dtw_bank_features.py (preprocessing and
// DTW) and the ranking, candidate and gate steps of
// recognize_with_template_bank() in dtw_template_bank_tools.py.
// tests/test_web_recognizer_parity.py runs both on the same queries and
// fails if they disagree. No DOM access here, so Node can import it.

/** Return finite [x, y] pairs from [x, y] arrays or {x, y} objects. */
export function coercePoints(points) {
  const out = [];
  for (const point of points || []) {
    if (point == null) continue;
    const x = Array.isArray(point) ? point[0] : point.x;
    const y = Array.isArray(point) ? point[1] : point.y;
    const fx = Number(x);
    const fy = Number(y);
    if (x == null || y == null || !Number.isFinite(fx) || !Number.isFinite(fy)) continue;
    out.push([fx, fy]);
  }
  return out;
}

/** Resample a polyline to n points evenly spaced along its arc length. */
export function resampleByArclength(points, n = 64) {
  if (points.length < 2) return points.slice();
  const dists = [0];
  for (let i = 1; i < points.length; i++) {
    const [ax, ay] = points[i - 1];
    const [bx, by] = points[i];
    dists.push(dists[i - 1] + Math.hypot(bx - ax, by - ay));
  }
  const total = dists[dists.length - 1];
  if (total <= 1e-12) return Array.from({ length: n }, () => points[0]);

  const out = [];
  let j = 0;
  for (let k = 0; k < n; k++) {
    const target = (total * k) / (n - 1);
    while (j < dists.length - 2 && dists[j + 1] < target) j++;
    const d0 = dists[j];
    const d1 = dists[j + 1];
    const [x0, y0] = points[j];
    const [x1, y1] = points[j + 1];
    const alpha = d1 <= d0 ? 0 : (target - d0) / (d1 - d0);
    out.push([x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)]);
  }
  return out;
}

/** Center on the centroid, scale by the max abs coordinate, then resample by arc length. */
export function normalizePoints(points, resampleLength = 64) {
  const clean = coercePoints(points);
  if (clean.length < 2) return clean;
  let sx = 0;
  let sy = 0;
  for (const [x, y] of clean) {
    sx += x;
    sy += y;
  }
  const cx = sx / clean.length;
  const cy = sy / clean.length;
  const centered = clean.map(([x, y]) => [x - cx, y - cy]);
  let maxAbs = 0;
  for (const [x, y] of centered) maxAbs = Math.max(maxAbs, Math.abs(x), Math.abs(y));
  const scale = Math.max(maxAbs, 1e-9);
  return resampleByArclength(
    centered.map(([x, y]) => [x / scale, y / scale]),
    resampleLength,
  );
}

/** DTW distance divided by len(a) + len(b); full search unless a Sakoe-Chiba window is given. */
export function dtwDistance(a, b, window = null) {
  const n = a.length;
  const m = b.length;
  if (n === 0 || m === 0) return Infinity;
  const band = window == null ? Math.max(n, m) : Math.max(Math.trunc(window), Math.abs(n - m));
  let prev = new Float64Array(m + 1).fill(Infinity);
  let curr = new Float64Array(m + 1).fill(Infinity);
  prev[0] = 0;
  for (let i = 1; i <= n; i++) {
    const lo = Math.max(1, i - band);
    const hi = Math.min(m, i + band);
    curr[lo - 1] = Infinity;
    if (hi < m) curr[hi + 1] = Infinity;
    const [ax, ay] = a[i - 1];
    for (let j = lo; j <= hi; j++) {
      const [bx, by] = b[j - 1];
      const cost = Math.hypot(ax - bx, ay - by);
      curr[j] = cost + Math.min(prev[j], curr[j - 1], prev[j - 1]);
    }
    [prev, curr] = [curr, prev];
  }
  return prev[m] / (n + m);
}

function compareStrings(a, b) {
  return a < b ? -1 : a > b ? 1 : 0;
}

export function marginRatio(best, second) {
  if (!Number.isFinite(best) || !Number.isFinite(second)) return 0;
  if (second <= 1e-9) return 0;
  return Math.max(0, Math.min(1, (second - best) / second));
}

/** Rank labels by their best template distance; ties break on (distance, template id). */
export function rankLabels(query, templates, { topK = 3, labels = null, window = null } = {}) {
  const wanted = labels ? new Set(labels.map(String)) : null;
  const scored = [];
  for (const template of templates) {
    const label = String(template.label ?? "");
    if (wanted && !wanted.has(label)) continue;
    const distance = dtwDistance(query, coercePoints(template.points), window);
    scored.push({ distance, id: String(template.id ?? ""), label });
  }
  scored.sort((p, q) => p.distance - q.distance || compareStrings(p.id, q.id));

  const bestByLabel = new Map();
  for (const item of scored) if (!bestByLabel.has(item.label)) bestByLabel.set(item.label, item);
  const ranked = [...bestByLabel.values()].sort(
    (p, q) => p.distance - q.distance || compareStrings(p.label, q.label),
  );

  const top = ranked.slice(0, Math.max(1, Math.trunc(topK))).map((item, index) => ({
    rank: index + 1,
    label: item.label,
    distance: item.distance,
    template_id: item.id,
  }));
  const best = top[0] || { label: "", distance: Infinity, template_id: "" };
  const second = ranked.length > 1 ? ranked[1] : null;
  const secondDistance = second ? second.distance : Infinity;
  const margin = second ? secondDistance - best.distance : Infinity;
  return {
    top,
    best_label: best.label,
    best_distance: best.distance,
    nearest_template: best.template_id,
    second_label: second ? second.label : "",
    second_distance: secondDistance,
    margin,
    margin_ratio: marginRatio(best.distance, secondDistance),
    n_templates_compared: scored.length,
  };
}

/** Distance-derived pseudo-confidence in [0, 1]; not a probability. */
export function confidenceFromDistance(distance, maxDistance) {
  if (!Number.isFinite(distance) || maxDistance <= 1e-9) return 0;
  return Math.max(0, Math.min(1, 1 - distance / maxDistance));
}

const RUNTIME_UNCERTAINTY_REASONS = {
  no_candidates: "dtw_no_candidates",
  high_distance: "dtw_distance_too_large",
  low_margin: "dtw_margin_too_small",
  low_confidence: "dtw_confidence_too_low",
};

const finite = (value) => (Number.isFinite(value) ? value : 0);

/**
 * Capture points -> {candidates, gate, ranking, query}, like the runtime node.
 *
 * Query points are never y-flipped: they must already be y-up, as the
 * runtime expects. The caller flips canvas coordinates.
 */
export function recognize(points, bank, options = {}) {
  const {
    topK = 3,
    maxDistance = 0.12,
    minMargin = 0.01,
    minConfidence = 0.3,
    labels = bank.labels,
    window = null,
  } = options;
  const resample = (bank.preprocess && bank.preprocess.resample_points) || 64;
  const query = normalizePoints(points, resample);
  const ranking = rankLabels(query, bank.templates, { topK, labels, window });

  const candidates = [];
  for (const item of ranking.top) {
    if (!Number.isFinite(item.distance)) continue;
    const margin = item.rank === 1 ? ranking.margin : item.distance - ranking.best_distance;
    candidates.push({
      rank: item.rank,
      label: item.label,
      confidence: confidenceFromDistance(item.distance, maxDistance),
      distance: item.distance,
      margin: finite(margin),
      template_id: item.template_id,
    });
  }

  let reason = "";
  if (!candidates.length || !Number.isFinite(ranking.best_distance)) reason = "no_candidates";
  else if (ranking.best_distance > maxDistance) reason = "high_distance";
  else if (Number.isFinite(ranking.margin) && ranking.margin < minMargin) reason = "low_margin";
  else if (candidates[0].confidence < minConfidence) reason = "low_confidence";

  const accepted = !reason;
  const gate = {
    accepted,
    uncertain: !accepted,
    uncertainty_reason: accepted ? "" : RUNTIME_UNCERTAINTY_REASONS[reason],
    top1_confidence: candidates.length ? candidates[0].confidence : 0,
    top2_confidence: candidates.length > 1 ? candidates[1].confidence : 0,
    best_distance: candidates.length ? finite(ranking.best_distance) : 0,
    margin: candidates.length ? finite(ranking.margin) : 0,
    max_distance: maxDistance,
    min_margin: minMargin,
    min_confidence: minConfidence,
  };
  return { candidates, gate, ranking, query };
}

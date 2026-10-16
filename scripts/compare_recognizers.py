#!/usr/bin/env python3
"""Compare stroke recognizers on the tracked template bank, clean and perturbed.

Every recognizer in ``colmag_ros.gesture_recognizers`` is scored with the same
leave-one-out protocol as ``scripts/eval_dtw_bank_loo.py``: each template is
held out once and classified against the others. The held-out stroke is also
classified after seeded synthetic perturbations (noise, rotation, truncation,
one-axis stretch), so methods that all score 100% on clean input can still be
told apart.

The perturbations are synthetic and the bank was drawn by one person with a
mouse. The results compare methods on this data; they are not accuracy on the
magnetic board or on other writers. Only the DTW recognizer runs in the ROS
pipeline; the others exist for this comparison.

Starts no ROS, GUI, serial or robot process, and writes nothing unless an
output path is given.

Usage::

    python3 scripts/compare_recognizers.py
    python3 scripts/compare_recognizers.py --methods protractor,features --repeats 3
    python3 scripts/compare_recognizers.py --repeats 3 --jobs 4 \\
        --svg-out docs/media/recognizer_robustness.svg --json-out outputs/recognizers.json
"""

from __future__ import annotations

import argparse
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO / "colmag_ros" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from colmag_ros.dtw_template_bank_tools import coerce_points, load_template_bank  # noqa: E402
from colmag_ros.gesture_recognizers import RECOGNIZERS, Template, make_recognizer  # noqa: E402
from colmag_ros.recognition_benchmark import PERTURBATIONS, evaluate_leave_one_out  # noqa: E402

DEFAULT_BANK = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"

Row = Dict[str, Any]


def bank_templates(bank: Dict[str, Any], per_label: Optional[int] = None) -> List[Template]:
    """Bank templates as (label, points); ``per_label`` keeps only the first N of each label."""
    kept: Dict[str, int] = {}
    out: List[Template] = []
    for template in bank.get("templates", []):
        label = str(template.get("label", ""))
        if per_label is not None and kept.get(label, 0) >= per_label:
            continue
        kept[label] = kept.get(label, 0) + 1
        out.append((label, coerce_points(template.get("points", []))))
    return out


def _evaluate(job: Tuple[str, List[Template], List[str], int]) -> List[Row]:
    key, templates, families, repeats = job
    return evaluate_leave_one_out(make_recognizer(key), templates, families, repeats)


def run(templates: List[Template], methods: Sequence[str], families: Sequence[str],
        repeats: int = 1, jobs: int = 1) -> List[Row]:
    work = [(key, templates, list(families), repeats) for key in methods]
    if jobs > 1 and len(work) > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            results = list(pool.map(_evaluate, work))
    else:
        results = [_evaluate(item) for item in work]
    return [row for rows in results for row in rows]


def _lookup(rows: Sequence[Row]) -> Dict[Tuple[str, str, float], Row]:
    return {(r["recognizer"], r["family"], r["level"]): r for r in rows}


def _recognizer_names(rows: Sequence[Row]) -> List[str]:
    names: List[str] = []
    for row in rows:
        if row["recognizer"] not in names:
            names.append(row["recognizer"])
    return names


def _families(rows: Sequence[Row]) -> List[str]:
    families: List[str] = []
    for row in rows:
        if row["family"] != "clean" and row["family"] not in families:
            families.append(row["family"])
    return families


def _levels(rows: Sequence[Row], family: str) -> List[float]:
    return sorted({r["level"] for r in rows if r["family"] == family})


def _level_text(family: str, level: float) -> str:
    if family == "rotation":
        return "%g°" % level
    if family == "truncation":
        return "%g%%" % (100 * level)
    if family == "stretch":
        return "%g×" % level
    return "σ=%g" % level


def format_markdown(rows: Sequence[Row]) -> str:
    table = _lookup(rows)
    names = _recognizer_names(rows)
    families = _families(rows)

    lines = [
        "## Recognizer comparison: leave-one-out on the tracked bank",
        "",
        "%d held-out queries clean and %d per perturbed condition. Top-1 accuracy "
        "unless noted; the perturbed columns use each family's strongest level."
        % (rows[0]["queries"], max(r["queries"] for r in rows)),
        "",
        "| Recognizer | Clean | Clean Top-3 | "
        + " | ".join("%s %s" % (family.capitalize(), _level_text(family, _levels(rows, family)[-1]))
                     for family in families)
        + " | Time per query |",
        "| --- " * (len(families) + 4) + "|",
    ]
    for name in names:
        clean = table[(name, "clean", 0.0)]
        cells = ["%.1f%%" % (100 * table[(name, f, _levels(rows, f)[-1])]["top1"]) for f in families]
        lines.append("| %s | %.1f%% | %.1f%% | %s | %.1f ms |" % (
            name, 100 * clean["top1"], 100 * clean["top3"], " | ".join(cells), clean["ms_per_query"]))

    for family in families:
        levels = _levels(rows, family)
        lines += [
            "",
            "### %s" % PERTURBATIONS[family][2],
            "",
            "| Recognizer | " + " | ".join(_level_text(family, level) for level in levels) + " |",
            "| --- " * (len(levels) + 1) + "|",
        ]
        for name in names:
            cells = ["%.1f%%" % (100 * table[(name, family, level)]["top1"]) for level in levels]
            lines.append("| %s | %s |" % (name, " | ".join(cells)))
    return "\n".join(lines) + "\n"


# Categorical slots 1-5 of the reference data-viz palette, light and dark.
# Each series also has its own marker shape, so color is never the only cue.
SERIES_LIGHT = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
SERIES_DARK = ("#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300")
MARKERS = ("circle", "square", "diamond", "triangle", "triangle-down", "plus")


def _marker(shape: str, x: float, y: float, cls: str) -> str:
    r = 4.0
    if shape == "circle":
        return '<circle class="%s" cx="%.1f" cy="%.1f" r="%.1f"/>' % (cls, x, y, r)
    if shape == "square":
        return '<rect class="%s" x="%.1f" y="%.1f" width="%.1f" height="%.1f" rx="1"/>' % (
            cls, x - r * 0.9, y - r * 0.9, 1.8 * r, 1.8 * r)
    if shape == "diamond":
        points = [(x, y - r * 1.25), (x + r * 1.25, y), (x, y + r * 1.25), (x - r * 1.25, y)]
    elif shape == "triangle":
        points = [(x, y - r * 1.2), (x + r * 1.15, y + r * 0.9), (x - r * 1.15, y + r * 0.9)]
    elif shape == "triangle-down":
        points = [(x, y + r * 1.2), (x + r * 1.15, y - r * 0.9), (x - r * 1.15, y - r * 0.9)]
    else:  # plus
        a, b = r * 1.3, r * 0.45
        points = [(x - b, y - a), (x + b, y - a), (x + b, y - b), (x + a, y - b), (x + a, y + b),
                  (x + b, y + b), (x + b, y + a), (x - b, y + a), (x - b, y + b), (x - a, y + b),
                  (x - a, y - b), (x - b, y - b)]
    return '<polygon class="%s" points="%s"/>' % (
        cls, " ".join("%.1f,%.1f" % point for point in points))


def format_svg(rows: Sequence[Row]) -> str:
    """Small multiples: one panel per perturbation family, Top-1 accuracy against level."""
    table = _lookup(rows)
    names = _recognizer_names(rows)
    families = _families(rows)

    panel_w, panel_h, gap = 230.0, 170.0, 28.0
    left, top, bottom = 44.0, 92.0, 30.0
    width = left + len(families) * panel_w + (len(families) - 1) * gap + 16.0
    height = top + panel_h + bottom

    css = [
        ".bg{fill:#fcfcfb}.t{fill:#0b0b0b}.s{fill:#52514e}.m{fill:#77766f}",
        ".grid{stroke:#e6e5e0;stroke-width:1}.axis{stroke:#bdbcb5;stroke-width:1}",
        "text{font-family:-apple-system,'Segoe UI',Helvetica,Arial,sans-serif}",
        "path.line{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round}",
    ]
    dark = [".bg{fill:#1a1a19}.t{fill:#f0efec}.s{fill:#c3c2b7}.m{fill:#9b9a92}",
            ".grid{stroke:#2e2e2c}.axis{stroke:#4a4a47}"]
    for i, (light_color, dark_color) in enumerate(zip(SERIES_LIGHT, SERIES_DARK)):
        css.append(".l%d{stroke:%s}.p%d{fill:%s;stroke:#fcfcfb;stroke-width:1.5}"
                   % (i, light_color, i, light_color))
        dark.append(".l%d{stroke:%s}.p%d{fill:%s;stroke:#1a1a19}" % (i, dark_color, i, dark_color))

    out = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.0f %.0f" width="%.0f" height="%.0f"'
        ' role="img" aria-labelledby="title desc">' % (width, height, width, height),
        "<title id=\"title\">Recognizer Top-1 accuracy under synthetic perturbations</title>",
        "<desc id=\"desc\">Leave-one-out on the tracked 8-symbol bank. One panel per perturbation "
        "family; x is perturbation strength, y is Top-1 accuracy from 0 to 100 percent. The "
        "same numbers are tabulated in docs/RECOGNIZER_COMPARISON.md.</desc>",
        "<style>%s@media (prefers-color-scheme:dark){%s}</style>" % ("".join(css), "".join(dark)),
        '<rect class="bg" width="100%" height="100%" rx="6"/>',
        '<text class="t" x="16" y="24" font-size="15" font-weight="600">'
        "Top-1 accuracy under synthetic perturbations</text>",
    ]

    legend_x = 16.0
    for i, name in enumerate(names):
        out.append(_marker(MARKERS[i % len(MARKERS)], legend_x + 5, 44, "p%d" % (i % len(MARKERS))))
        out.append('<text class="s" x="%.1f" y="48" font-size="12">%s</text>'
                   % (legend_x + 14, _escape(name)))
        legend_x += 26 + 7.0 * len(name)

    for p, family in enumerate(families):
        x0 = left + p * (panel_w + gap)
        levels = [0.0] + _levels(rows, family)

        def px(index: int) -> float:
            return x0 + 8 + index * (panel_w - 16) / max(1, len(levels) - 1)

        def py(value: float) -> float:
            return top + panel_h - value * panel_h

        for tick in (0.0, 0.25, 0.5, 0.75, 1.0):
            out.append('<line class="grid" x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f"/>'
                       % (x0, x0 + panel_w, py(tick), py(tick)))
            if p == 0:
                out.append('<text class="m" x="%.1f" y="%.1f" font-size="11" text-anchor="end">'
                           "%d%%</text>" % (x0 - 6, py(tick) + 4, int(100 * tick)))
        out.append('<line class="axis" x1="%.1f" x2="%.1f" y1="%.1f" y2="%.1f"/>'
                   % (x0, x0 + panel_w, py(0.0), py(0.0)))
        for index, level in enumerate(levels):
            label = "clean" if index == 0 else _level_text(family, level)
            out.append('<text class="m" x="%.1f" y="%.1f" font-size="11" text-anchor="middle">%s</text>'
                       % (px(index), py(0.0) + 16, _escape(label)))
        out.append('<text class="t" x="%.1f" y="%.1f" font-size="12" font-weight="600">%s</text>'
                   % (x0, top - 12, _escape(PERTURBATIONS[family][2])))

        for i, name in enumerate(names):
            values = [table[(name, "clean", 0.0)]["top1"]] + [
                table[(name, family, level)]["top1"] for level in levels[1:]]
            d = " ".join("%s%.1f,%.1f" % ("M" if k == 0 else "L", px(k), py(v))
                         for k, v in enumerate(values))
            out.append('<path class="line l%d" d="%s"/>' % (i % len(MARKERS), d))
            for k, value in enumerate(values):
                out.append(_marker(MARKERS[i % len(MARKERS)], px(k), py(value), "p%d" % (i % len(MARKERS))))
    out.append("</svg>")
    return "\n".join(out) + "\n"


def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("--bank", type=Path, default=DEFAULT_BANK,
                        help="template bank JSON (default: the tracked bank)")
    parser.add_argument("--methods", default=",".join(RECOGNIZERS),
                        help="comma-separated recognizers: %s" % ", ".join(RECOGNIZERS))
    parser.add_argument("--families", default=",".join(PERTURBATIONS),
                        help="comma-separated perturbations: %s" % ", ".join(PERTURBATIONS))
    parser.add_argument("--repeats", type=int, default=1,
                        help="perturbed queries per template and level (default 1)")
    parser.add_argument("--per-label", type=int,
                        help="use only the first N templates of each label (quick runs)")
    parser.add_argument("--jobs", type=int, default=1, help="recognizers to run in parallel")
    parser.add_argument("--markdown-out", type=Path, help="write the Markdown tables here")
    parser.add_argument("--svg-out", type=Path, help="write the robustness chart here")
    parser.add_argument("--json-out", type=Path, help="write every result row here as JSON")
    parser.add_argument("--from-json", type=Path,
                        help="re-render the tables and chart from a saved --json-out file instead "
                             "of running the benchmark")
    args = parser.parse_args(argv)

    methods = [m for m in args.methods.split(",") if m]
    families = [f for f in args.families.split(",") if f]
    unknown = [m for m in methods if m not in RECOGNIZERS] + [f for f in families if f not in PERTURBATIONS]
    if unknown:
        parser.error("unknown recognizer or perturbation: %s" % ", ".join(unknown))

    if args.from_json:
        rows = json.loads(args.from_json.read_text(encoding="utf-8"))
    else:
        templates = bank_templates(load_template_bank(args.bank), args.per_label)
        rows = run(templates, methods, families, args.repeats, args.jobs)
    markdown = format_markdown(rows)
    sys.stdout.write(markdown)

    for path, text in ((args.markdown_out, markdown), (args.svg_out, format_svg(rows)),
                       (args.json_out, json.dumps(rows, indent=2) + "\n")):
        if path:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Evaluate a DTW template bank against existing dashboard runtime samples.

M104-B2c offline feasibility tooling. Loads a template bank JSON, extracts
``raw_symbol_capture.points`` from runtime sample JSONs, ranks every query
against the bank, and reports accuracy / confusion / margin / threshold-grid
behavior. Runtime queries are never y-flipped; the seed-side y-flip decision
lives in the bank. Reports go under ignored
``outputs/classifier_artifacts/dtw_eval/``. No ROS/runtime is touched.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from dtw_dataset_io import (  # noqa: E402
    ensure_output_dir,
    load_runtime_sample,
    write_csv,
    write_json,
)
from dtw_template_bank_tools import (  # noqa: E402
    DEFAULT_EVAL_DIR,
    decide_acceptance,
    is_bad_intended_label,
    load_template_bank,
    parse_labels_csv,
    preprocess_query_for_bank,
    rank_labels_for_query,
)

MAX_DISTANCE_GRID = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40)
MIN_MARGIN_GRID = (0.0, 0.02, 0.05, 0.10)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Offline M104-B2c: evaluate a DTW template bank on runtime samples."
    )
    parser.add_argument("--template-bank", required=True)
    parser.add_argument("--runtime-samples-root",
                        default="outputs/classifier_training/runtime_samples")
    parser.add_argument("--labels", default="1,2,3",
                        help="Comma-separated intended labels to score (default: 1,2,3)")
    parser.add_argument("--out-dir", default="",
                        help="Report dir; default outputs/classifier_artifacts/dtw_eval/eval_<ts>")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--min-margin", type=float, default=0.02,
                        help="Primary accept gate: absolute DTW distance margin")
    parser.add_argument("--max-distance", type=float, default=0.30,
                        help="Primary accept gate: max nearest DTW distance")
    parser.add_argument("--print-rows", action="store_true")
    parser.add_argument("--include-bad", action="store_true",
                        help="Also score bad/ambiguous samples (reported separately, never in accuracy)")
    parser.add_argument("--include-invalid-sessions", action="store_true",
                        help="Include session dirs whose name contains INVALID (default: skipped)")
    return parser


def _repo_path(path):
    value = Path(path)
    return value if value.is_absolute() else REPO / value


def collect_runtime_sample_files(root, include_invalid_sessions=False):
    base = Path(root)
    files = []
    for path in sorted(base.rglob("*.json")):
        relative = path.relative_to(base)
        if not include_invalid_sessions and any("invalid" in part.lower() for part in relative.parts[:-1]):
            continue
        files.append(path)
    return files


def build_eval_row(path, session, bank, args):
    record = load_runtime_sample(path)
    intended = str(record.get("intended_label", ""))
    query = preprocess_query_for_bank(record.get("points", []), bank)
    ranking = rank_labels_for_query(query, bank, top_k=args.top_k,
                                    labels=parse_labels_csv(args.labels))
    gate = decide_acceptance(ranking["best_distance"], ranking["margin"],
                             args.max_distance, args.min_margin)
    is_bad = is_bad_intended_label(intended)
    correct = bool(intended and not is_bad and ranking["best_label"] == intended)
    row = {
        "file": str(path),
        "session": session,
        "intended_label": intended,
        "is_bad_sample": is_bad,
        "point_count": len(record.get("points", [])),
        "predicted_label": ranking["best_label"],
        "best_distance": ranking["best_distance"],
        "nearest_template": ranking["nearest_template"],
        "second_label": ranking["second_label"],
        "second_distance": ranking["second_distance"],
        "margin": ranking["margin"],
        "margin_ratio": ranking["margin_ratio"],
        "top_k": json.dumps(ranking["top"], sort_keys=True),
        "correct": correct,
        "false_accepted": bool(gate["accepted"] and not is_bad and intended
                               and ranking["best_label"] != intended),
        "bad_accepted": bool(gate["accepted"] and is_bad),
    }
    row.update(gate)
    return row


def threshold_grid_summary(rows):
    grid = []
    scored = [row for row in rows if not row["is_bad_sample"]]
    bad = [row for row in rows if row["is_bad_sample"]]
    for max_distance in MAX_DISTANCE_GRID:
        for min_margin in MIN_MARGIN_GRID:
            accepted = [row for row in scored
                        if decide_acceptance(row["best_distance"], row["margin"],
                                             max_distance, min_margin)["accepted"]]
            wrong = [row for row in accepted if not row["correct"]]
            bad_accepted = [row for row in bad
                            if decide_acceptance(row["best_distance"], row["margin"],
                                                 max_distance, min_margin)["accepted"]]
            grid.append({
                "max_distance": max_distance,
                "min_margin": min_margin,
                "accepted": len(accepted),
                "uncertain": len(scored) - len(accepted),
                "accepted_correct": len(accepted) - len(wrong),
                "false_accepted": len(wrong),
                "bad_accepted": len(bad_accepted),
            })
    return grid


def _margin_stats(rows):
    values = [float(row["margin"]) for row in rows
              if row["margin"] is not None and row["margin"] != float("inf")]
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
        "median": statistics.median(values),
    }


def summarize_rows(rows, labels):
    scored = [row for row in rows if not row["is_bad_sample"]]
    bad = [row for row in rows if row["is_bad_sample"]]
    confusion = Counter(
        "%s -> %s" % (row["intended_label"], row["predicted_label"])
        for row in scored if not row["correct"]
    )
    per_label = {}
    for label in labels:
        group = [row for row in scored if row["intended_label"] == label]
        per_label[label] = {
            "count": len(group),
            "top1_correct": sum(1 for row in group if row["correct"]),
            "top1_accuracy": (sum(1 for row in group if row["correct"]) / len(group)) if group else None,
            "accepted": sum(1 for row in group if row["accepted"]),
            "false_accepted": sum(1 for row in group if row["false_accepted"]),
            "margin": _margin_stats(group),
        }
    return {
        "scored_samples": len(scored),
        "top1_correct": sum(1 for row in scored if row["correct"]),
        "top1_accuracy": (sum(1 for row in scored if row["correct"]) / len(scored)) if scored else None,
        "accepted": sum(1 for row in scored if row["accepted"]),
        "uncertain": sum(1 for row in scored if not row["accepted"]),
        "false_accepted": sum(1 for row in scored if row["false_accepted"]),
        "confusion_pairs": dict(confusion),
        "confusion_2_3": sum(count for pair, count in confusion.items()
                             if pair in ("2 -> 3", "3 -> 2")),
        "false_accept_3_as_1_or_2": sum(1 for row in scored
                                        if row["intended_label"] == "3" and row["false_accepted"]
                                        and row["predicted_label"] in ("1", "2")),
        "per_label": per_label,
        "margin_correct": _margin_stats([row for row in scored if row["correct"]]),
        "margin_incorrect": _margin_stats([row for row in scored if not row["correct"]]),
        "bad_samples": {
            "count": len(bad),
            "accepted": sum(1 for row in bad if row["accepted"]),
            "predicted_labels": dict(Counter(row["predicted_label"] for row in bad)),
            "margin": _margin_stats(bad),
        },
    }


def _print_rows(rows):
    for row in rows:
        print("%-14s intended=%-4s pred=%-4s d=%.4f margin=%s acc=%s %s" % (
            Path(row["file"]).name[:14],
            row["intended_label"],
            row["predicted_label"],
            row["best_distance"],
            ("inf" if row["margin"] == float("inf") else "%.4f" % row["margin"]),
            "Y" if row["accepted"] else "n",
            "" if row["correct"] or row["is_bad_sample"] else "<- WRONG",
        ))


def main(argv=None):
    args = build_parser().parse_args(argv)
    labels = parse_labels_csv(args.labels)
    out_dir = ensure_output_dir(
        _repo_path(args.out_dir or (DEFAULT_EVAL_DIR / time.strftime("eval_%Y%m%d_%H%M%S"))),
        [REPO / "outputs"],
    )
    try:
        bank = load_template_bank(_repo_path(args.template_bank))
    except Exception as exc:
        print("ERROR: failed to load template bank: %s" % exc, file=sys.stderr)
        return 2
    samples_root = _repo_path(args.runtime_samples_root)
    if not samples_root.is_dir():
        print("ERROR: runtime samples root not found: %s" % samples_root, file=sys.stderr)
        return 2

    rows = []
    load_errors = []
    skipped_other = 0
    for path in collect_runtime_sample_files(samples_root, args.include_invalid_sessions):
        session = path.parent.name
        try:
            row = build_eval_row(path, session, bank, args)
        except Exception as exc:
            load_errors.append({"file": str(path), "error": str(exc)})
            continue
        if row["is_bad_sample"] and not args.include_bad:
            skipped_other += 1
            continue
        if not row["is_bad_sample"] and row["intended_label"] not in labels:
            skipped_other += 1
            continue
        rows.append(row)
    if not rows:
        print("ERROR: no runtime samples matched labels %s under %s"
              % (",".join(labels), samples_root), file=sys.stderr)
        return 2

    summary = summarize_rows(rows, labels)
    summary.update({
        "template_bank": str(_repo_path(args.template_bank)),
        "bank_y_flip": bool(bank["preprocess"].get("y_flip")),
        "bank_profile": str(bank["preprocess"].get("profile", "")),
        "labels": labels,
        "primary_gate": {"max_distance": args.max_distance, "min_margin": args.min_margin},
        "skipped_out_of_scope": skipped_other,
        "load_errors": load_errors,
        "threshold_grid": threshold_grid_summary(rows),
    })
    write_csv(out_dir / "eval_rows.csv", rows)
    write_json(out_dir / "eval_rows.json", rows)
    write_json(out_dir / "summary.json", summary)

    if args.print_rows:
        _print_rows(rows)
    print("eval_out_dir: %s" % out_dir)
    print("bank_y_flip: %s  scored: %d  top1_accuracy: %s" % (
        summary["bank_y_flip"], summary["scored_samples"],
        ("%.4f" % summary["top1_accuracy"]) if summary["top1_accuracy"] is not None else "n/a"))
    print("accepted: %d  uncertain: %d  false_accepted: %d" % (
        summary["accepted"], summary["uncertain"], summary["false_accepted"]))
    print("confusion_pairs: %s" % json.dumps(summary["confusion_pairs"], sort_keys=True))
    print("bad_samples: %s" % json.dumps(summary["bad_samples"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

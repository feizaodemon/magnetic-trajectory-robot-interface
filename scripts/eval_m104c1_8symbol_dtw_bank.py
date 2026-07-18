#!/usr/bin/env python3
"""M104-C1 offline held-out evaluation for the 8-symbol DTW template bank.

Recognition-only. This script does NOT touch ROS, Gazebo, GUI, serial, the real
board, the task dispatcher, the task-command topic, or any robot execution path.
It only measures whether the existing ``dtw_template_bank`` backend can separate
the C1 target vocabulary ``1,2,3,V,O,X,A,C`` on mouse seeds.

Because no runtime samples exist for ``V/O/X/A/C``, the existing
``scripts/eval_dtw_bank_on_runtime_samples.py`` cannot score them. Instead this
tool does a deterministic held-out split of the mouse seeds under
``outputs/classifier_training/seeds/<label>/*.json``: the earlier seeds per
label become templates, the later seeds become held-out queries. Both templates
and queries share the same y-flip + preprocess profile, so this is an offline
mouse-seed cross-validation, NOT a runtime or real-board PASS.

Reports go under ignored ``outputs/classifier_artifacts/m104c1_8symbol_eval/``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import Counter
from pathlib import Path
from urllib.parse import quote

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from dtw_dataset_io import (  # noqa: E402
    ensure_output_dir,
    load_standalone_seed,
    write_csv,
    write_json,
)
from dtw_template_bank_tools import (  # noqa: E402
    build_template_bank,
    build_template_record,
    confidence_from_distance,
    decide_acceptance,
    flip_y_points,
    parse_labels_csv,
    preprocess_query_for_bank,
    rank_labels_for_query,
)

# C1 target vocabulary. Recognition-only; no executable semantics are bound.
C1_TARGET_LABELS = ["1", "2", "3", "V", "O", "X", "A", "C"]
C1_EVAL_DIR = Path("outputs") / "classifier_artifacts" / "m104c1_8symbol_eval"

# C1 keeps the reviewed B2d/G2 gate so offline numbers stay comparable.
DEFAULT_MAX_DISTANCE = 0.12
DEFAULT_MIN_MARGIN = 0.01
DEFAULT_MIN_CONFIDENCE = 0.30


def build_parser():
    parser = argparse.ArgumentParser(
        description="Offline M104-C1: held-out 8-symbol DTW recognition-only evaluation."
    )
    parser.add_argument("--seeds-root", default="outputs/classifier_training/seeds")
    parser.add_argument("--labels", default=",".join(C1_TARGET_LABELS),
                        help="Comma-separated recognition labels (default: 1,2,3,V,O,X,A,C)")
    parser.add_argument("--holdout-fraction", type=float, default=0.4,
                        help="Fraction of each label's sorted seeds held out as queries")
    parser.add_argument("--out-dir", default="",
                        help="Report dir; default outputs/classifier_artifacts/m104c1_8symbol_eval/eval_<ts>")
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--max-distance", type=float, default=DEFAULT_MAX_DISTANCE)
    parser.add_argument("--min-margin", type=float, default=DEFAULT_MIN_MARGIN)
    parser.add_argument("--min-confidence", type=float, default=DEFAULT_MIN_CONFIDENCE)
    parser.add_argument("--preprocess-profile", default="mouse_centroid_maxabs_resample64")
    flip = parser.add_mutually_exclusive_group()
    flip.add_argument("--y-flip", dest="y_flip", action="store_true", default=True)
    flip.add_argument("--no-y-flip", dest="y_flip", action="store_false")
    parser.add_argument("--print-rows", action="store_true")
    return parser


def _repo_path(path):
    value = Path(path)
    return value if value.is_absolute() else REPO / value


def collect_label_seeds(seeds_root, labels):
    """Return {label: [sorted seed paths]} restricted to requested labels."""
    root = _repo_path(seeds_root)
    per_label = {}
    for label in labels:
        label_dir = root / quote(label, safe="-_.~")
        per_label[label] = sorted(label_dir.glob("*.json")) if label_dir.is_dir() else []
    return per_label


def split_templates_and_queries(seed_paths, holdout_fraction):
    """Deterministic split: earlier seeds are templates, later seeds are queries.

    Keeps at least one template and one query when the label has >= 2 seeds.
    """
    total = len(seed_paths)
    if total < 2:
        return list(seed_paths), []
    n_query = max(1, int(round(total * holdout_fraction)))
    n_query = min(n_query, total - 1)
    return seed_paths[: total - n_query], seed_paths[total - n_query:]


def build_bank(template_pairs, args):
    labels = parse_labels_csv(args.labels)
    templates = []
    per_label_index = {label: 0 for label in labels}
    for label, path in template_pairs:
        seed = load_standalone_seed(path)
        templates.append(build_template_record(
            label, seed["points"],
            source_file=str(path), source_type="mouse_seed",
            y_flip=args.y_flip, profile=args.preprocess_profile,
            source_stroke_count=seed.get("stroke_count"),
            index=per_label_index[label],
        ))
        per_label_index[label] += 1
    bank = build_template_bank(
        templates, labels=labels, profile=args.preprocess_profile, y_flip=args.y_flip,
        generated_by="scripts/eval_m104c1_8symbol_dtw_bank.py",
    )
    return bank


def score_query(path, intended, bank, args):
    seed = load_standalone_seed(path)
    raw = flip_y_points(seed["points"]) if args.y_flip else seed["points"]
    query = preprocess_query_for_bank(raw, bank)
    ranking = rank_labels_for_query(query, bank, top_k=args.top_k,
                                    labels=parse_labels_csv(args.labels))
    confidence = confidence_from_distance(ranking["best_distance"], args.max_distance)
    gate = decide_acceptance(ranking["best_distance"], ranking["margin"],
                             args.max_distance, args.min_margin,
                             top1_confidence=confidence, min_confidence=args.min_confidence)
    predicted = ranking["best_label"]
    correct = bool(predicted == intended)
    row = {
        "file": str(path),
        "intended_label": intended,
        "predicted_label": predicted,
        "best_distance": ranking["best_distance"],
        "second_label": ranking["second_label"],
        "second_distance": ranking["second_distance"],
        "margin": ranking["margin"],
        "confidence": confidence,
        "top_k": json.dumps(ranking["top"], sort_keys=True),
        "correct": correct,
        "accepted": bool(gate["accepted"]),
        "uncertain": bool(gate["uncertain"]),
        "uncertainty_reason": gate["uncertainty_reason"],
        "false_accepted": bool(gate["accepted"] and not correct),
    }
    return row


def summarize(rows, labels, template_counts, query_counts):
    confusion = Counter(
        "%s -> %s" % (r["intended_label"], r["predicted_label"])
        for r in rows if not r["correct"]
    )
    per_label = {}
    for label in labels:
        group = [r for r in rows if r["intended_label"] == label]
        n = len(group)
        per_label[label] = {
            "templates": template_counts.get(label, 0),
            "test_samples": query_counts.get(label, 0),
            "top1_correct": sum(1 for r in group if r["correct"]),
            "top1_accuracy": (sum(1 for r in group if r["correct"]) / n) if n else None,
            "accepted": sum(1 for r in group if r["accepted"]),
            "uncertain": sum(1 for r in group if r["uncertain"]),
            "false_accepted": sum(1 for r in group if r["false_accepted"]),
        }
    n = len(rows)
    return {
        "labels": labels,
        "scored_samples": n,
        "top1_correct": sum(1 for r in rows if r["correct"]),
        "top1_accuracy": (sum(1 for r in rows if r["correct"]) / n) if n else None,
        "accepted": sum(1 for r in rows if r["accepted"]),
        "uncertain": sum(1 for r in rows if r["uncertain"]),
        "false_accepted": sum(1 for r in rows if r["false_accepted"]),
        "confusion_pairs": dict(confusion),
        "worst_confusions": confusion.most_common(5),
        "per_label": per_label,
    }


def main(argv=None):
    args = build_parser().parse_args(argv)
    labels = parse_labels_csv(args.labels)
    per_label_seeds = collect_label_seeds(args.seeds_root, labels)

    missing = [label for label in labels if not per_label_seeds[label]]
    if missing:
        print("ERROR: no seeds found for labels: %s under %s"
              % (",".join(missing), args.seeds_root), file=sys.stderr)
        print("Collect mouse seeds first (see docs/VALIDATION.md and README.md).",
              file=sys.stderr)
        return 2

    template_pairs = []
    query_pairs = []
    template_counts = {}
    query_counts = {}
    for label in labels:
        templates, queries = split_templates_and_queries(
            per_label_seeds[label], args.holdout_fraction)
        template_counts[label] = len(templates)
        query_counts[label] = len(queries)
        template_pairs.extend((label, p) for p in templates)
        query_pairs.extend((label, p) for p in queries)

    if not query_pairs:
        print("ERROR: every label has < 2 seeds; cannot hold out queries.", file=sys.stderr)
        return 2

    bank = build_bank(template_pairs, args)
    rows = [score_query(path, label, bank, args) for label, path in query_pairs]

    summary = summarize(rows, labels, template_counts, query_counts)
    summary["gate"] = {
        "max_distance": args.max_distance,
        "min_margin": args.min_margin,
        "min_confidence": args.min_confidence,
    }
    summary["holdout_fraction"] = args.holdout_fraction
    summary["y_flip"] = bool(args.y_flip)
    summary["preprocess_profile"] = args.preprocess_profile

    out_dir = ensure_output_dir(
        _repo_path(args.out_dir or (C1_EVAL_DIR / time.strftime("eval_%Y%m%d_%H%M%S"))),
        [REPO / "outputs"],
    )
    write_csv(out_dir / "eval_rows.csv", rows)
    write_json(out_dir / "eval_rows.json", rows)
    write_json(out_dir / "summary.json", summary)

    if args.print_rows:
        for r in rows:
            print("%-3s -> %-3s d=%.4f margin=%s conf=%.3f acc=%s %s" % (
                r["intended_label"], r["predicted_label"], r["best_distance"],
                ("inf" if r["margin"] == float("inf") else "%.4f" % r["margin"]),
                r["confidence"], "Y" if r["accepted"] else "n",
                "" if r["correct"] else "<- WRONG"))

    print("eval_out_dir: %s" % out_dir)
    print("labels: %s" % ",".join(labels))
    print("templates_per_label: %s" % json.dumps(template_counts, sort_keys=True))
    print("test_per_label: %s" % json.dumps(query_counts, sort_keys=True))
    print("scored: %d  top1_accuracy: %s" % (
        summary["scored_samples"],
        ("%.4f" % summary["top1_accuracy"]) if summary["top1_accuracy"] is not None else "n/a"))
    print("accepted: %d  uncertain: %d  false_accepted: %d" % (
        summary["accepted"], summary["uncertain"], summary["false_accepted"]))
    print("worst_confusions: %s" % json.dumps(summary["worst_confusions"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

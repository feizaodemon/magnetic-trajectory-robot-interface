#!/usr/bin/env python3
"""Leave-one-out evaluation of the tracked DTW template bank.

Each template in turn is removed from the bank and used as the query against
the remaining templates, with the same ranking and runtime gates as the
recognizer node. The script reports Top-1 and Top-3 accuracy, how many
queries the runtime gates accept, a confusion matrix, and per-query latency.

Unlike the other offline tools, this one needs only files tracked in the
repository, so anyone can reproduce the numbers in the README. It starts no
ROS, GUI, serial or robot process and writes nothing unless ``--json-out`` is
given.

The bank was built from one person's mouse-drawn seeds, so the numbers say
how well the templates separate from each other, not how well the recognizer
generalizes to other writers or to the magnetic board.

Usage::

    python3 scripts/eval_dtw_bank_loo.py
    python3 scripts/eval_dtw_bank_loo.py --min-top1 0.95   # CI gate
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from _repo import TRACKED_BANK  # also puts colmag_ros/src on sys.path

from colmag_ros.dtw_template_bank_tools import (
    DEFAULT_MAX_DISTANCE,
    DEFAULT_MIN_CONFIDENCE,
    DEFAULT_MIN_MARGIN,
    coerce_points,
    confidence_from_distance,
    decide_acceptance,
    load_template_bank,
    rank_labels_for_query,
)

# (query points, bank, top_k, labels) -> ranking with the keys that
# rank_labels_for_query returns. Other recognizers can be scored by passing
# their own function to leave_one_out.
Ranker = Callable[..., Dict[str, Any]]

DEFAULT_BANK = TRACKED_BANK


def leave_one_out(bank: Dict[str, Any],
                  labels: Optional[Sequence[str]] = None,
                  max_distance: float = DEFAULT_MAX_DISTANCE,
                  min_margin: float = DEFAULT_MIN_MARGIN,
                  min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                  rank: Ranker = rank_labels_for_query) -> Dict[str, Any]:
    """Score every template against the bank without it, and summarize.

    See ``score_queries`` for the per-query rows.
    """
    templates = list(bank.get("templates", []))
    labels = _bank_labels(templates, labels)
    rows = score_queries(bank, labels, max_distance, min_margin, min_confidence, rank)
    return summarize(rows, labels, bank_templates=len(templates))


def _bank_labels(templates: Sequence[Dict[str, Any]],
                 labels: Optional[Sequence[str]]) -> List[str]:
    if labels is None:
        labels = sorted({str(t.get("label", "")) for t in templates})
    return [str(label) for label in labels]


def score_queries(bank: Dict[str, Any],
                  labels: Optional[Sequence[str]] = None,
                  max_distance: float = DEFAULT_MAX_DISTANCE,
                  min_margin: float = DEFAULT_MIN_MARGIN,
                  min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                  rank: Ranker = rank_labels_for_query) -> List[Dict[str, Any]]:
    """Rank each template against the bank without it; one row per query.

    Template points are already preprocessed with the bank profile, so they
    go to ``rank`` unchanged, exactly as a preprocessed runtime query would.
    ``rank`` must return ``top``, ``best_label``, ``best_distance`` and
    ``margin`` as ``rank_labels_for_query`` does; the gates use the last two.
    """
    templates = list(bank.get("templates", []))
    labels = _bank_labels(templates, labels)

    rows: List[Dict[str, Any]] = []
    for index, template in enumerate(templates):
        truth = str(template.get("label", ""))
        if truth not in labels:
            continue
        rest = dict(bank, templates=templates[:index] + templates[index + 1:])
        query = coerce_points(template.get("points", []))

        start = time.perf_counter()
        ranking = rank(query, rest, top_k=3, labels=labels)
        latency_ms = (time.perf_counter() - start) * 1000.0

        top_labels = [item["label"] for item in ranking["top"]]
        confidence = confidence_from_distance(ranking["best_distance"], max_distance)
        gate = decide_acceptance(ranking["best_distance"], ranking["margin"],
                                 max_distance, min_margin,
                                 top1_confidence=confidence,
                                 min_confidence=min_confidence)
        rows.append({
            "id": str(template.get("id", "")),
            "label": truth,
            "predicted": ranking["best_label"],
            "top3": top_labels,
            "best_distance": ranking["best_distance"],
            "margin": ranking["margin"],
            "accepted": gate["accepted"],
            "reject_reason": gate["uncertainty_reason"],
            "latency_ms": latency_ms,
        })
    return rows


def summarize(rows: Sequence[Dict[str, Any]], labels: Sequence[str],
              bank_templates: int) -> Dict[str, Any]:
    n = len(rows)
    top1 = sum(r["predicted"] == r["label"] for r in rows)
    top3 = sum(r["label"] in r["top3"] for r in rows)
    accepted = [r for r in rows if r["accepted"]]
    accepted_correct = sum(r["predicted"] == r["label"] for r in accepted)

    confusion = {truth: {pred: 0 for pred in labels} for truth in labels}
    for r in rows:
        if r["predicted"] in confusion[r["label"]]:
            confusion[r["label"]][r["predicted"]] += 1

    per_label = {}
    for label in labels:
        mine = [r for r in rows if r["label"] == label]
        per_label[label] = {
            "queries": len(mine),
            "top1": sum(r["predicted"] == label for r in mine) / len(mine) if mine else 0.0,
        }

    latencies = sorted(r["latency_ms"] for r in rows)
    reject_reasons: Dict[str, int] = {}
    for r in rows:
        if not r["accepted"]:
            reject_reasons[r["reject_reason"]] = reject_reasons.get(r["reject_reason"], 0) + 1

    return {
        "bank_templates": bank_templates,
        "labels": list(labels),
        "queries": n,
        "top1_accuracy": top1 / n if n else 0.0,
        "top3_accuracy": top3 / n if n else 0.0,
        "gate_accept_rate": len(accepted) / n if n else 0.0,
        "accepted_precision": accepted_correct / len(accepted) if accepted else 0.0,
        "reject_reasons": reject_reasons,
        "latency_ms": {
            "median": statistics.median(latencies) if latencies else 0.0,
            "p95": latencies[min(n - 1, int(0.95 * n))] if latencies else 0.0,
            "max": latencies[-1] if latencies else 0.0,
        },
        "per_label": per_label,
        "confusion": confusion,
        "errors": [r for r in rows if r["predicted"] != r["label"]],
    }


def format_markdown(result: Dict[str, Any]) -> str:
    lat = result["latency_ms"]
    lines = [
        "## DTW template bank: leave-one-out",
        "",
        "%d queries, each against the other %d templates."
        % (result["queries"], result["bank_templates"] - 1),
        "",
        "| Metric | Value |",
        "| --- | --- |",
        "| Top-1 accuracy | %.1f%% |" % (100 * result["top1_accuracy"]),
        "| Top-3 accuracy | %.1f%% |" % (100 * result["top3_accuracy"]),
        "| Accepted by runtime gates | %.1f%% |" % (100 * result["gate_accept_rate"]),
        "| Precision of accepted | %.1f%% |" % (100 * result["accepted_precision"]),
        "| Latency per query (median / p95) | %.1f / %.1f ms |" % (lat["median"], lat["p95"]),
        "",
        "| Label | Queries | Top-1 |",
        "| --- | --- | --- |",
    ]
    for label, stats in result["per_label"].items():
        lines.append("| `%s` | %d | %.1f%% |" % (label, stats["queries"], 100 * stats["top1"]))

    labels = result["labels"]
    lines += ["", "Confusion matrix (rows: true label, columns: predicted):", "",
              "| | " + " | ".join("`%s`" % label for label in labels) + " |",
              "| --- " * (len(labels) + 1) + "|"]
    for truth in labels:
        cells = [str(result["confusion"][truth][pred]) for pred in labels]
        lines.append("| `%s` | " % truth + " | ".join(cells) + " |")

    if result["reject_reasons"]:
        reasons = ", ".join("%s: %d" % item for item in sorted(result["reject_reasons"].items()))
        lines += ["", "Gate rejections: %s." % reasons]
    if result["errors"]:
        lines += ["", "Misrecognized: " + ", ".join(
            "%s as `%s`" % (r["id"], r["predicted"]) for r in result["errors"]) + "."]
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bank", type=Path, default=DEFAULT_BANK,
                        help="template bank JSON (default: the tracked bank)")
    parser.add_argument("--json-out", type=Path,
                        help="also write the full result, per-query rows included, as JSON")
    parser.add_argument("--min-top1", type=float,
                        help="exit 1 if Top-1 accuracy is below this fraction")
    args = parser.parse_args(argv)

    bank = load_template_bank(args.bank)
    labels = bank.get("allowed_labels") or bank.get("labels")
    result = leave_one_out(bank, labels=labels)
    sys.stdout.write(format_markdown(result))

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    if args.min_top1 is not None and result["top1_accuracy"] < args.min_top1:
        sys.stderr.write("Top-1 accuracy %.3f is below --min-top1 %.3f\n"
                         % (result["top1_accuracy"], args.min_top1))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Benchmark the optional Sakoe-Chiba window on the tracked 8-symbol bank.

Recognition-only and ROS-free. Each bank template is used once as a query
against the other 123 templates (leave-one-out), first with the full DTW
search the runtime uses by default, then with each window width. A window is
an approximation: distances can only grow, so the table reports top-1
accuracy and how many queries got a different top-3 label order.

Usage: python3 scripts/bench_dtw_ranking.py [--bank PATH] [--windows 4,8,16]
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PACKAGE_SRC = REPO / "colmag_ros" / "src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from colmag_ros.dtw_template_bank_tools import (
    coerce_points,
    load_template_bank,
    rank_labels_for_query,
)

DEFAULT_BANK = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"


def leave_one_out(bank):
    templates = bank["templates"]
    for index, template in enumerate(templates):
        rest = dict(bank, templates=templates[:index] + templates[index + 1:])
        yield str(template["label"]), coerce_points(template["points"]), rest


def run(bank, window):
    latencies, top_labels, correct = [], [], 0
    for label, query, rest in leave_one_out(bank):
        start = time.perf_counter()
        ranking = rank_labels_for_query(query, rest, top_k=3, window=window)
        latencies.append((time.perf_counter() - start) * 1000.0)
        top_labels.append([item["label"] for item in ranking["top"]])
        correct += ranking["best_label"] == label
    return {
        "top_labels": top_labels,
        "median_ms": statistics.median(latencies),
        "p95_ms": sorted(latencies)[int(0.95 * (len(latencies) - 1))],
        "top1": correct / len(latencies),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bank", type=Path, default=DEFAULT_BANK)
    parser.add_argument("--windows", default="4,8,16",
                        help="comma-separated Sakoe-Chiba widths, in resampled points")
    args = parser.parse_args(argv)

    bank = load_template_bank(args.bank)
    print("bank: %s (%d templates, labels %s)\n" % (
        args.bank.name, len(bank["templates"]), ",".join(bank["labels"])))

    full = run(bank, None)
    print("| DTW search | median ms/query | p95 ms | speed-up | LOO top-1 | top-3 order differs |")
    print("| --- | ---: | ---: | ---: | ---: | ---: |")
    print("| full (default) | %.1f | %.1f | 1.0x | %.1f%% | - |" % (
        full["median_ms"], full["p95_ms"], 100.0 * full["top1"]))
    for width in (int(w) for w in args.windows.split(",") if w):
        row = run(bank, width)
        differs = sum(a != b for a, b in zip(row["top_labels"], full["top_labels"]))
        print("| window=%d | %.1f | %.1f | %.1fx | %.1f%% | %d of %d |" % (
            width, row["median_ms"], row["p95_ms"], full["median_ms"] / row["median_ms"],
            100.0 * row["top1"], differs, len(row["top_labels"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Assemble the static browser recognizer demo in ``web/recognizer``.

Copies the page files and writes two data files next to them: the tracked
DTW bank (minified) and ``config.json`` with the runtime gate defaults and
the symbol display names, read from the Python package so the page cannot
drift from the robot-side values. The GitHub Pages workflow publishes the
output folder; locally, serve it with ``python3 -m http.server``.
"""

import argparse
import json
import shutil
from pathlib import Path

from _repo import REPO, TRACKED_BANK

from colmag_ros import dtw_template_bank_tools as bank_tools
from colmag_ros.symbol_semantics import SYMBOL_SEMANTICS

WEB_SRC = REPO / "web" / "recognizer"
PAGE_FILES = ("index.html", "style.css", "app.js", "recognizer.js")


def demo_config(bank):
    labels = [str(label) for label in bank["labels"]]
    return {
        "bank_file": "bank.json",
        "bank_name": TRACKED_BANK.stem,
        "labels": labels,
        "top_k": 3,
        "max_distance": bank_tools.DEFAULT_MAX_DISTANCE,
        "min_margin": bank_tools.DEFAULT_MIN_MARGIN,
        "min_confidence": bank_tools.DEFAULT_MIN_CONFIDENCE,
        "semantics": {
            label: {
                "display_name": SYMBOL_SEMANTICS[label]["display_name"],
                "description": SYMBOL_SEMANTICS[label]["description"],
            }
            for label in labels
            if label in SYMBOL_SEMANTICS
        },
    }


def build(out_dir):
    bank = bank_tools.load_template_bank(TRACKED_BANK)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in PAGE_FILES:
        shutil.copyfile(WEB_SRC / name, out_dir / name)
    (out_dir / "bank.json").write_text(json.dumps(bank, separators=(",", ":")), encoding="utf-8")
    (out_dir / "config.json").write_text(json.dumps(demo_config(bank), indent=2), encoding="utf-8")
    # GitHub Pages: serve files as they are, no Jekyll processing.
    (out_dir / ".nojekyll").write_text("", encoding="utf-8")
    return out_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=REPO / "outputs" / "web_demo",
                        help="output folder (default: outputs/web_demo)")
    args = parser.parse_args()
    out = build(args.out)
    print("wrote %s" % out)


if __name__ == "__main__":
    main()

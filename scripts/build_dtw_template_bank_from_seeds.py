#!/usr/bin/env python3
"""Build a DTW template bank from existing user-handwritten mouse seed JSONs.

M104-B2c offline feasibility tooling. Reads seeds from
``outputs/classifier_training/seeds/<label>/*.json``, converts them into DTW
template records, and writes a template bank JSON under ignored
``outputs/classifier_artifacts/dtw_bank/``. No ROS/runtime wiring, no synthetic
augmentation, no executable mapping; default labels stay ``1,2,3``.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from urllib.parse import quote

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from dtw_dataset_io import (  # noqa: E402
    ensure_output_dir,
    load_standalone_seed,
    write_json,
)
from dtw_template_bank_tools import (  # noqa: E402
    DEFAULT_BANK_DIR,
    DEFAULT_PREPROCESS_PROFILE,
    PREPROCESS_PROFILES,
    build_template_bank,
    build_template_record,
    parse_labels_csv,
)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Offline M104-B2c: build a DTW template bank from mouse seed JSONs."
    )
    parser.add_argument("--seeds-root", default="outputs/classifier_training/seeds")
    parser.add_argument("--labels", default="1,2,3",
                        help="Comma-separated recognition labels (default: 1,2,3)")
    parser.add_argument("--max-per-label", type=int, default=0,
                        help="Cap templates per label; 0 keeps all seeds")
    parser.add_argument("--out", default=str(DEFAULT_BANK_DIR / "mouse_seeded_dtw_bank.json"))
    flip = parser.add_mutually_exclusive_group()
    flip.add_argument("--y-flip", dest="y_flip", action="store_true", default=True,
                      help="Flip seed y (canvas y-down -> runtime y-up); default on")
    flip.add_argument("--no-y-flip", dest="y_flip", action="store_false")
    parser.add_argument("--resample-points", type=int, default=0,
                        help="Override profile resample length; 0 uses the profile value")
    parser.add_argument("--preprocess-profile", default=DEFAULT_PREPROCESS_PROFILE,
                        choices=sorted(PREPROCESS_PROFILES))
    parser.add_argument("--source-type", default="mouse_seed")
    return parser


def _repo_path(path):
    value = Path(path)
    return value if value.is_absolute() else REPO / value


def collect_seed_files(seeds_root, labels):
    """Return sorted (label, path) pairs restricted to the requested labels."""
    root = Path(seeds_root)
    pairs = []
    for label in labels:
        label_dir = root / quote(label, safe="-_.~")
        if not label_dir.is_dir():
            continue
        for path in sorted(label_dir.glob("*.json")):
            pairs.append((label, path))
    return pairs


def build_bank_from_seeds(args):
    labels = parse_labels_csv(args.labels)
    templates = []
    per_label_index = {label: 0 for label in labels}
    skipped = []
    for label, path in collect_seed_files(_repo_path(args.seeds_root), labels):
        if args.max_per_label > 0 and per_label_index[label] >= args.max_per_label:
            continue
        try:
            seed = load_standalone_seed(path)
            template = build_template_record(
                label,
                seed["points"],
                source_file=str(path),
                source_type=args.source_type,
                y_flip=args.y_flip,
                profile=args.preprocess_profile,
                index=per_label_index[label],
                resample_points=args.resample_points or None,
                source_stroke_count=seed.get("stroke_count"),
            )
        except Exception as exc:
            skipped.append({"file": str(path), "error": str(exc)})
            continue
        templates.append(template)
        per_label_index[label] += 1
    bank = build_template_bank(
        templates,
        labels=labels,
        profile=args.preprocess_profile,
        y_flip=args.y_flip,
        resample_points=args.resample_points or None,
        generated_by="scripts/build_dtw_template_bank_from_seeds.py",
    )
    return bank, per_label_index, skipped


def main(argv=None):
    args = build_parser().parse_args(argv)
    out_path = _repo_path(args.out)
    ensure_output_dir(out_path.parent, [REPO / "outputs"])
    try:
        bank, per_label_counts, skipped = build_bank_from_seeds(args)
    except Exception as exc:
        print("ERROR: failed to build template bank: %s" % exc, file=sys.stderr)
        return 2
    if not bank["templates"]:
        print("ERROR: no usable seeds found under %s for labels %s"
              % (args.seeds_root, ",".join(parse_labels_csv(args.labels))), file=sys.stderr)
        return 2
    write_json(out_path, bank)
    print("template_bank: %s" % out_path)
    print("y_flip: %s  profile: %s" % (bank["preprocess"]["y_flip"], bank["preprocess"]["profile"]))
    print("per_label_counts: %s" % json.dumps(per_label_counts, sort_keys=True))
    if skipped:
        print("skipped_files: %d" % len(skipped))
        for item in skipped:
            print("  skipped: %s (%s)" % (item["file"], item["error"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

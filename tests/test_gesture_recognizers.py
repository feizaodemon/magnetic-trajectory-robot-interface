"""Unit tests for the alternative recognizers and the robustness benchmark.

Pure Python: no ROS, GUI, serial board or robot process is started.
"""

import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from colmag_ros import gesture_recognizers as gr
from colmag_ros import recognition_benchmark as rb
from colmag_ros.dtw_template_bank_tools import (
    coerce_points,
    load_template_bank,
    rank_labels_for_query,
)

REPO = Path(__file__).resolve().parents[1]
BANK_PATH = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"
SCRIPTS = REPO / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import compare_recognizers  # noqa: E402


@pytest.fixture(scope="module")
def bank():
    return load_template_bank(BANK_PATH)


@pytest.fixture(scope="module")
def small_set(bank):
    return compare_recognizers.bank_templates(bank, per_label=3)


def _rotate(points, degrees):
    a = math.radians(degrees)
    return [(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)) for x, y in points]


def test_dtw_recognizer_matches_runtime_bank_ranking(bank):
    # The recognizer normalizes every stroke it is given, so the runtime side
    # gets the same normalized query and templates; the ranking rule and the
    # distances must then agree exactly.
    recognizer = gr.DTWRecognizer()
    templates = bank["templates"]
    for index in (0, 40, 90):
        rest = templates[:index] + templates[index + 1:]
        normalized_rest = dict(bank, templates=[
            dict(t, points=recognizer.preprocess(coerce_points(t["points"]))) for t in rest])
        query = coerce_points(templates[index]["points"])
        runtime = rank_labels_for_query(recognizer.preprocess(query), normalized_rest, top_k=8)

        recognizer.fit([(t["label"], coerce_points(t["points"])) for t in rest])
        ranking = recognizer.rank(query)

        assert [label for label, _ in ranking] == [item["label"] for item in runtime["top"]]
        assert ranking[0][1] == pytest.approx(runtime["best_distance"])


@pytest.mark.parametrize("key", sorted(gr.RECOGNIZERS))
def test_every_recognizer_classifies_clean_held_out_templates(key, small_set):
    rows = rb.evaluate_leave_one_out(gr.make_recognizer(key), small_set, families=[])
    assert [(row["family"], row["queries"]) for row in rows] == [("clean", len(small_set))]
    assert rows[0]["top1"] >= 0.9
    assert rows[0]["top3"] == 1.0


@pytest.mark.parametrize("key", sorted(gr.RECOGNIZERS))
def test_rank_lists_each_label_once_best_first(key, small_set):
    recognizer = gr.make_recognizer(key).fit(small_set[1:])
    ranking = recognizer.rank(small_set[0][1])
    labels = [label for label, _ in ranking]
    distances = [distance for _, distance in ranking]
    assert sorted(labels) == sorted({label for label, _ in small_set})
    assert distances == sorted(distances)


def test_unknown_recognizer_key_raises():
    with pytest.raises(KeyError):
        gr.make_recognizer("nope")


def test_protractor_absorbs_rotation_only_within_its_limit(small_set):
    label, points = small_set[0]
    recognizer = gr.ProtractorRecognizer(max_rotation_deg=45.0)
    template = recognizer.preprocess(points)

    within = recognizer.distance(recognizer.preprocess(_rotate(points, 30)), template)
    beyond = recognizer.distance(recognizer.preprocess(_rotate(points, 120)), template)
    assert within == pytest.approx(0.0, abs=1e-6)
    assert beyond > 0.5


def test_pdollar_ignores_stroke_direction(small_set):
    _, points = small_set[5]
    recognizer = gr.PDollarRecognizer()
    forward = recognizer.preprocess(points)
    backward = recognizer.preprocess(list(reversed(points)))
    other = recognizer.preprocess(small_set[-1][1])
    assert recognizer.distance(backward, forward) < 0.1 * recognizer.distance(other, forward)


def test_stroke_features_are_translation_and_scale_invariant(small_set):
    _, points = small_set[7]
    moved = [(3.0 * x + 10.0, 3.0 * y - 4.0) for x, y in points]
    original = gr.stroke_features(points)
    assert len(original) == gr.DIRECTION_BINS + 8 + gr.GRID_SIZE ** 2
    assert gr.stroke_features(moved) == pytest.approx(original, abs=1e-9)
    assert len(gr.stroke_features([(0.0, 0.0)])) == len(original)


def test_derivative_sequence_keeps_length_and_handles_short_input():
    line = [(float(i), 2.0 * i) for i in range(5)]
    slopes = gr.derivative_sequence(line)
    assert len(slopes) == len(line)
    assert all(s == pytest.approx((1.0, 2.0)) for s in slopes)
    assert gr.derivative_sequence([(0.0, 0.0), (1.0, 1.0)]) == [(0.0, 0.0), (0.0, 0.0)]


def test_as_points_drops_malformed_and_non_finite_items():
    assert gr.as_points([(1, 2), [3, "4"], (None, 1), (1,), (float("nan"), 0)]) == [(1.0, 2.0), (3.0, 4.0)]


def test_perturbations_are_seeded_and_keep_strokes_usable():
    points = [(math.cos(t / 10.0), math.sin(t / 10.0)) for t in range(64)]
    for family, (_, levels, _) in rb.PERTURBATIONS.items():
        for level in levels:
            seed = rb.query_seed(0, family, level, 0)
            first = rb.perturbed_query(points, family, level, random.Random(seed))
            again = rb.perturbed_query(points, family, level, random.Random(seed))
            assert first == again
            assert len(first) >= 2
    assert rb.perturbed_query(points, "clean", 0.0, random.Random(0)) == points
    assert len(rb.truncate(points, 0.25, random.Random(0))) == 48


def test_rotation_perturbation_preserves_centroid_and_size():
    points = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)]
    rotated = rb.rotate(points, 90.0, random.Random(1))
    cx = sum(x for x, _ in rotated) / 3
    cy = sum(y for _, y in rotated) / 3
    assert (cx, cy) == pytest.approx((4.0 / 3, 1.0 / 3))
    assert gr.path_length(rotated) == pytest.approx(gr.path_length(points))


def test_conditions_start_clean_then_follow_family_levels():
    plan = rb.conditions(["rotation"])
    assert plan[0] == ("clean", 0.0)
    assert plan[1:] == [("rotation", level) for level in rb.PERTURBATIONS["rotation"][1]]


def test_cli_writes_markdown_svg_and_json(tmp_path, capsys):
    md, svg, js = tmp_path / "r.md", tmp_path / "r.svg", tmp_path / "r.json"
    code = compare_recognizers.main([
        "--methods", "protractor,features", "--families", "rotation,truncation",
        "--per-label", "2", "--markdown-out", str(md), "--svg-out", str(svg), "--json-out", str(js),
    ])
    assert code == 0
    assert "| Protractor ($1) | 100.0%" in capsys.readouterr().out
    assert md.read_text(encoding="utf-8").startswith("## Recognizer comparison")
    root = ET.fromstring(svg.read_text(encoding="utf-8"))
    assert root.tag.endswith("svg")
    rows = json.loads(js.read_text(encoding="utf-8"))
    assert {row["recognizer"] for row in rows} == {"Protractor ($1)", "Feature k-NN"}
    assert len(rows) == 2 * len(rb.conditions(["rotation", "truncation"]))


def test_cli_rejects_unknown_method():
    with pytest.raises(SystemExit):
        compare_recognizers.main(["--methods", "bogus"])

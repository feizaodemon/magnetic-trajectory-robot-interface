"""The browser recognizer (web/recognizer/recognizer.js) must match the Python runtime path.

Runs the JavaScript port under Node on seeded queries built from the tracked
bank (exact templates, noisy, rotated, truncated and scribbled strokes, so
both accepted and rejected outcomes are covered) and compares candidates and
gate decisions with ``recognize_with_template_bank``. Skips when Node is not
installed, unless ``COLMAG_REQUIRE_NODE=1``.
"""

import json
import math
import os
import random
import shutil
import subprocess
from pathlib import Path

import pytest

from colmag_ros import dtw_template_bank_tools as bank_tools

REPO = Path(__file__).resolve().parents[1]
BANK = REPO / "colmag_ros" / "config" / "dtw_banks" / "m104c1_8symbol_mouse_seeded_yflip.json"
RECOGNIZER_JS = REPO / "web" / "recognizer" / "recognizer.js"
LABELS = ["1", "2", "3", "V", "O", "X", "A", "C"]

NODE_RUNNER = """
import { readFileSync } from "node:fs";
import { pathToFileURL } from "node:url";
const [modPath, bankPath, queriesPath] = process.argv.slice(2);
const { recognize } = await import(pathToFileURL(modPath).href);
const bank = JSON.parse(readFileSync(bankPath, "utf8"));
const { labels, queries } = JSON.parse(readFileSync(queriesPath, "utf8"));
const out = queries.map((q) => {
  const { candidates, gate } = recognize(q, bank, { labels });
  return { candidates, gate };
});
process.stdout.write(JSON.stringify(out));
"""


def _node():
    node = shutil.which("node")
    if node is None:
        if os.environ.get("COLMAG_REQUIRE_NODE") == "1":
            pytest.fail("node is required (COLMAG_REQUIRE_NODE=1) but not installed")
        pytest.skip("node is not installed")
    return node


def _queries(bank):
    rng = random.Random(20261016)
    queries = []
    # Every fourth template keeps the pure-Python side at about 15 s.
    for template in bank["templates"][::4]:
        pts = [(float(x), float(y)) for x, y in template["points"]]
        queries.append(pts)
        queries.append([(x + rng.gauss(0, 0.08), y + rng.gauss(0, 0.08)) for x, y in pts])
        angle = math.radians(rng.choice([-50, -30, 30, 50]))
        c, s = math.cos(angle), math.sin(angle)
        queries.append([(c * x - s * y, s * x + c * y) for x, y in pts])
        queries.append(pts[: max(2, int(len(pts) * rng.uniform(0.3, 0.7)))])
    for _ in range(20):
        queries.append([(rng.uniform(-1, 1), rng.uniform(-1, 1)) for _ in range(rng.randint(2, 30))])
    # Raw canvas-like coordinates: offset and scale must not matter.
    queries.append([(300 + 80 * x, -200 + 80 * y) for x, y in bank["templates"][0]["points"]])
    return queries


def _close(a, b):
    return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)


def test_js_recognizer_matches_python(tmp_path):
    node = _node()
    state = bank_tools.load_template_bank_state(BANK, LABELS)
    assert state["status"] == "ready"
    queries = _queries(state["bank"])
    queries_file = tmp_path / "queries.json"
    queries_file.write_text(json.dumps({"labels": LABELS, "queries": queries}))
    runner = tmp_path / "runner.mjs"
    runner.write_text(NODE_RUNNER)
    proc = subprocess.run(
        [node, str(runner), str(RECOGNIZER_JS), str(BANK), str(queries_file)],
        check=True, capture_output=True, text=True, timeout=300,
    )
    js_results = json.loads(proc.stdout)
    assert len(js_results) == len(queries)

    outcomes = set()
    for query, js in zip(queries, js_results):
        candidates, gate, _ = bank_tools.recognize_with_template_bank(
            query, state,
            top_k=3,
            max_distance=bank_tools.DEFAULT_MAX_DISTANCE,
            min_margin=bank_tools.DEFAULT_MIN_MARGIN,
            min_confidence=bank_tools.DEFAULT_MIN_CONFIDENCE,
        )
        assert [c["label"] for c in js["candidates"]] == [c["label"] for c in candidates]
        for jc, pc in zip(js["candidates"], candidates):
            assert jc["rank"] == pc["rank"]
            assert jc["template_id"] == pc["template_id"]
            for key in ("distance", "confidence", "margin"):
                assert _close(jc[key], pc[key]), (key, jc[key], pc[key])
        assert js["gate"]["accepted"] == gate["accepted"]
        assert js["gate"]["uncertainty_reason"] == gate["uncertainty_reason"]
        for key in ("top1_confidence", "top2_confidence", "best_distance", "margin"):
            assert _close(js["gate"][key], gate[key]), (key, js["gate"][key], gate[key])
        outcomes.add(gate["uncertainty_reason"] or "accepted")

    # The queries must exercise both acceptance and rejection.
    assert "accepted" in outcomes
    assert "dtw_distance_too_large" in outcomes


def test_demo_config_uses_runtime_gate_defaults():
    import importlib.util
    import sys

    sys.path.insert(0, str(REPO / "scripts"))
    spec = importlib.util.spec_from_file_location("build_web_demo", REPO / "scripts" / "build_web_demo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = module.demo_config(bank_tools.load_template_bank(BANK))
    assert config["labels"] == LABELS
    assert config["max_distance"] == bank_tools.DEFAULT_MAX_DISTANCE
    assert config["min_margin"] == bank_tools.DEFAULT_MIN_MARGIN
    assert config["min_confidence"] == bank_tools.DEFAULT_MIN_CONFIDENCE
    assert set(config["semantics"]) == set(LABELS)

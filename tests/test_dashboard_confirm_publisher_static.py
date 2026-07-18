"""Static/unit checks for the preview-confirm helper module.

No ROS master, Gazebo, GUI, serial board, FR3, or robot runtime is started.
"""

import ast
import json
import unittest
from pathlib import Path

from colmag_ros.scripts.dashboard_confirm_publisher import (
    PreviewConfirmPublisher,
    external_payload_is_fresh,
)
from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
    build_confirmed_label_payload,
    candidate_payload_key,
    should_suppress_repeated_confirm,
)


REPO = Path(__file__).resolve().parents[1]
HELPER = REPO / "colmag_ros" / "scripts" / "dashboard_confirm_publisher.py"


class _String:
    def __init__(self, data=""):
        self.data = data


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Logger:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def logwarn(self, *args):
        self.warnings.append(args)

    def loginfo(self, *args):
        self.infos.append(args)


def _helper(logger=None):
    return PreviewConfirmPublisher(
        string_factory=_String,
        build_confirmed_label_payload=build_confirmed_label_payload,
        should_suppress_repeated_confirm=should_suppress_repeated_confirm,
        candidate_payload_key=candidate_payload_key,
        logger=logger,
    )


class PreviewConfirmHelperBehaviorTests(unittest.TestCase):
    def test_publishes_payload_without_ros_master(self):
        pub = _Publisher()
        result = _helper().publish_rank_one(
            confirm_pub=pub,
            integrated_confirm_enabled=True,
            trajectory_candidates=[{"rank": 1, "label": "2", "confidence": 0.91}],
            last_confirm_key="",
        )
        self.assertTrue(result.published)
        self.assertEqual(result.label, "2")
        self.assertEqual(result.selected_rank, 1)
        self.assertEqual(result.command_intent, "CONFIRM_RANK_1")
        payload = json.loads(pub.messages[0].data)
        self.assertEqual(payload["label"], "2")
        self.assertEqual(payload["confirmed_by"], "magnetic_dashboard_preview_dwell")
        self.assertFalse(payload["controls_real_robot"])
        self.assertEqual(payload["source_topic"], "/colmag/symbol_capture")

    def test_gate_and_empty_candidate_paths_do_not_publish(self):
        logger = _Logger()
        pub = _Publisher()
        disabled = _helper(logger).publish_rank_one(pub, False, [{"rank": 1, "label": "2"}], "")
        empty = _helper(logger).publish_rank_one(pub, True, [], "")
        self.assertFalse(disabled.published)
        self.assertEqual(disabled.reason, "disabled")
        self.assertFalse(empty.published)
        self.assertEqual(empty.reason, "no_candidate")
        self.assertEqual(pub.messages, [])
        self.assertTrue(logger.warnings)


def _external_payload(sample_id="cap_1", sequence_id=7):
    return {
        "backend": "dtw_template_bank",
        "sample_id": sample_id,
        "sequence_id": sequence_id,
        "candidates": [
            {"rank": 1, "label": "2", "confidence": 0.88, "task": "HEXAGON_TRAJECTORY"},
            {"rank": 2, "label": "1", "confidence": 0.20},
            {"rank": 3, "label": "3", "confidence": 0.10},
        ],
    }


class ExternalConfirmPathTests(unittest.TestCase):
    """M104-G3a-F1: confirm the external dtw_template_bank symbol_candidates
    payload authoritatively, preserving recognizer metadata."""

    def test_external_confirm_preserves_backend_and_metadata(self):
        pub = _Publisher()
        result = _helper().publish_external_payload(
            confirm_pub=pub,
            integrated_confirm_enabled=True,
            candidate_payload=_external_payload(),
            last_confirm_key="",
            expected_sample_id="cap_1",
            expected_sequence_id=7,
        )
        self.assertTrue(result.published)
        self.assertEqual(result.label, "2")
        payload = json.loads(pub.messages[0].data)
        self.assertEqual(payload["backend"], "dtw_template_bank")
        self.assertTrue(payload["confirmed"])
        self.assertEqual(payload["confirmed_by"], "magnetic_dashboard_dwell")
        self.assertFalse(payload["controls_real_robot"])
        self.assertEqual(payload["label"], "2")
        self.assertEqual(payload["selected_rank"], 1)
        self.assertEqual(payload["sample_id"], "cap_1")
        self.assertEqual(payload["sequence_id"], 7)
        self.assertEqual(payload["source_topic"], "/colmag/symbol_capture")
        # Per-candidate metadata (task/command_intent/template) is preserved via
        # the embedded candidates list; the dispatcher maps label -> task.
        self.assertEqual(payload["candidates"][0]["task"], "HEXAGON_TRAJECTORY")

    def test_stale_external_candidates_not_confirmed(self):
        logger = _Logger()
        pub = _Publisher()
        # sample_id mismatch -> stale, not published.
        stale_sid = _helper(logger).publish_external_payload(
            pub, True, _external_payload(sample_id="cap_1"), "",
            expected_sample_id="cap_2", expected_sequence_id=7,
        )
        # sequence_id mismatch -> stale, not published.
        stale_seq = _helper(logger).publish_external_payload(
            pub, True, _external_payload(sequence_id=7), "",
            expected_sample_id="cap_1", expected_sequence_id=9,
        )
        self.assertFalse(stale_sid.published)
        self.assertEqual(stale_sid.reason, "stale")
        self.assertFalse(stale_seq.published)
        self.assertEqual(stale_seq.reason, "stale")
        self.assertEqual(pub.messages, [])

    def test_external_confirm_gate_and_empty_paths(self):
        pub = _Publisher()
        disabled = _helper().publish_external_payload(pub, False, _external_payload(), "")
        empty = _helper().publish_external_payload(pub, True, {"candidates": []}, "")
        self.assertEqual(disabled.reason, "disabled")
        self.assertEqual(empty.reason, "no_candidate")
        self.assertEqual(pub.messages, [])

    def test_unknown_expected_ids_skip_stale_guard(self):
        pub = _Publisher()
        result = _helper().publish_external_payload(
            pub, True, _external_payload(), "",
            expected_sample_id=None, expected_sequence_id=None,
        )
        self.assertTrue(result.published)

    def test_external_payload_is_fresh_helper(self):
        payload = _external_payload(sample_id="cap_1", sequence_id=7)
        self.assertTrue(external_payload_is_fresh(payload, "cap_1", 7))
        self.assertTrue(external_payload_is_fresh(payload, None, None))
        self.assertFalse(external_payload_is_fresh(payload, "cap_9", 7))
        self.assertFalse(external_payload_is_fresh(payload, "cap_1", 9))


class PreviewConfirmHelperStaticTests(unittest.TestCase):
    def test_helper_module_is_under_1000_lines(self):
        self.assertLess(HELPER.read_text().count("\n") + 1, 1000)

    def test_helper_functions_are_under_120_lines(self):
        tree = ast.parse(HELPER.read_text())
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                end = getattr(node, "end_lineno", node.lineno)
                self.assertLessEqual(
                    end - node.lineno + 1,
                    120,
                    "%s is too long" % node.name,
                )


if __name__ == "__main__":
    unittest.main()

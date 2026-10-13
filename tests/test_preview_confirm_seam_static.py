"""Bounded static/unit checks for the trajectory-preview confirm seam.

No ROS master, Gazebo, Tk display, serial board, FR3, or hardware is started.
These tests exercise the real-board / trajectory-preview C-confirm publish path
(`_publish_preview_confirmed_label`) on a headless fake node built with
``object.__new__`` so no rospy/tkinter is imported. The wiring tests build a
full node on the fakes in ``dashboard_harness``.
"""

import json
import unittest
from pathlib import Path

from magnetic_trajectory_dashboard_node import (
    MagneticTrajectoryDashboardNode,
    build_confirmed_label_payload,
    candidate_payload_key,
    should_suppress_repeated_confirm,
)
from colmag_ros.dashboard_confirm_publisher import PreviewConfirmPublisher
from dashboard_harness import DashboardHarness, default_config


REPO = Path(__file__).resolve().parents[1]
HELPER = REPO / "colmag_ros" / "src" / "colmag_ros" / "dashboard_confirm_publisher.py"
CONFIRM_PARAMS = {
    "integrated_dashboard_confirm_enabled": True,
    "publish_confirmed_label": True,
}


class _String:
    def __init__(self, data=""):
        self.data = data


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _Rospy:
    def __init__(self):
        self.warnings = []
        self.infos = []

    def logwarn(self, *args):
        self.warnings.append(args)

    def loginfo(self, *args):
        self.infos.append(args)


class _Var:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value


class _Root:
    def after(self, _delay, fn=None):
        # Run the scheduled UI callback synchronously so confirm_var is exercised.
        if callable(fn):
            fn()


def _preview_node(*, confirm_pub_enabled=True, integrated=True, candidates=None):
    node = object.__new__(MagneticTrajectoryDashboardNode)
    node.rospy = _Rospy()
    node.String = _String
    node.root = _Root()
    node.confirm_var = _Var()
    node.confirm_pub = _Publisher() if confirm_pub_enabled else None
    node.preview_confirm_publisher = PreviewConfirmPublisher(
        string_factory=_String,
        build_confirmed_label_payload=build_confirmed_label_payload,
        should_suppress_repeated_confirm=should_suppress_repeated_confirm,
        candidate_payload_key=candidate_payload_key,
        logger=node.rospy,
    )
    node.config = default_config(integrated_confirm_enabled=integrated)
    node.current_candidate_payload = None
    node.trajectory_candidates = candidates if candidates is not None else [
        {"rank": 1, "label": "2", "confidence": 0.88, "distance": 0.1},
        {"rank": 2, "label": "1", "confidence": 0.40, "distance": 0.5},
    ]
    node.last_confirm_key = ""
    node.last_confirmed_label = ""
    node.last_selected_rank = None
    node.last_command_intent = ""
    # Keep dashboard-state publishing inert for the unit test.
    node._publish_dashboard_state = lambda: None
    return node


class PreviewConfirmPublishTests(unittest.TestCase):
    def test_publishes_confirmed_label_when_both_gates_on(self):
        node = _preview_node()
        node._publish_preview_confirmed_label()
        self.assertEqual(len(node.confirm_pub.published), 1)
        payload = json.loads(node.confirm_pub.published[0].data)
        self.assertEqual(payload["label"], "2")
        self.assertEqual(payload["selected_rank"], 1)
        self.assertTrue(payload["confirmed"])
        self.assertEqual(payload["confirmed_by"], "magnetic_dashboard_preview_dwell")
        self.assertFalse(payload["controls_real_robot"])
        self.assertFalse(payload["test_only"])
        self.assertEqual(payload["source_topic"], "/colmag/symbol_capture")
        self.assertIn("candidates", payload)
        self.assertIn("timestamp", payload)
        # No fabricated top-3: candidate list is whatever preview produced.
        self.assertEqual(len(payload["candidates"]), 2)
        # Never carries a task field (task_dispatcher_node owns label->task).
        self.assertNotIn("task", payload)

    def test_no_publish_when_publish_confirmed_label_disabled(self):
        # publish_confirmed_label=false => confirm_pub is None (never created).
        node = _preview_node(confirm_pub_enabled=False)
        node._publish_preview_confirmed_label()  # must be a safe no-op
        self.assertEqual(node.last_confirmed_label, "")

    def test_no_publish_when_integrated_confirm_disabled(self):
        node = _preview_node(integrated=False)
        node._publish_preview_confirmed_label()
        self.assertEqual(node.confirm_pub.published, [])

    def test_no_candidate_is_safe_no_op_with_log(self):
        node = _preview_node(candidates=[])
        node._publish_preview_confirmed_label()
        self.assertEqual(node.confirm_pub.published, [])
        self.assertTrue(node.rospy.warnings)

    def test_repeated_confirm_is_suppressed_until_candidates_change(self):
        node = _preview_node()
        node._publish_preview_confirmed_label()
        node._publish_preview_confirmed_label()  # same candidates -> suppressed
        self.assertEqual(len(node.confirm_pub.published), 1)
        # A new stroke / new candidates re-arms the confirm.
        node.trajectory_candidates = [{"rank": 1, "label": "3", "confidence": 0.9, "distance": 0.1}]
        node._publish_preview_confirmed_label()
        self.assertEqual(len(node.confirm_pub.published), 2)
        self.assertEqual(json.loads(node.confirm_pub.published[1].data)["label"], "3")


def test_preview_c_confirm_publishes_the_shared_helper_payload_once(monkeypatch):
    harness = DashboardHarness(monkeypatch, candidate_preview_enabled=True, **CONFIRM_PARAMS)
    harness.send_sample_and_candidates()
    expected = _Publisher()
    PreviewConfirmPublisher(
        string_factory=_String,
        build_confirmed_label_payload=build_confirmed_label_payload,
        should_suppress_repeated_confirm=should_suppress_repeated_confirm,
        candidate_payload_key=candidate_payload_key,
    ).publish_rank_one(expected, True, harness.node.trajectory_candidates, "")

    harness.press_key("c")
    harness.press_key("c")  # same candidates -> suppressed

    published = harness.published_json(harness.node.config.confirmed_label_topic)
    expected_payload = json.loads(expected.published[0].data)
    assert len(published) == 1
    for payload in (published[0], expected_payload):
        payload.pop("timestamp")
    assert published[0] == expected_payload


def test_helper_module_is_small():
    assert HELPER.read_text().count("\n") + 1 < 1000


def test_dashboard_has_no_task_command_publisher(monkeypatch):
    # Even with every publish gate on, the dashboard never publishes task commands.
    harness = DashboardHarness(
        monkeypatch, candidate_preview_enabled=True, control_mode_enabled=True,
        publish_task_command=True, **CONFIRM_PARAMS)
    assert harness.node.config.task_command_topic not in harness.publisher_topics()
    assert any(
        level == "warn" and "never publishes /colmag/task_command" in text
        for level, text in harness.rospy.logs
    )


if __name__ == "__main__":
    unittest.main()

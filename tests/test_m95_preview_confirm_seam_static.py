"""Bounded static/unit checks for the M95 trajectory-preview confirm seam.

No ROS master, Gazebo, Tk display, serial board, FR3, or hardware is started.
These tests exercise the real-board / trajectory-preview C-confirm publish path
(`_publish_preview_confirmed_label`) on a headless fake node built with
``object.__new__`` so no rospy/tkinter is imported.
"""

import json
import unittest
from pathlib import Path

from colmag_ros.scripts.magnetic_trajectory_dashboard_node import (
    MagneticTrajectoryDashboardNode,
    build_confirmed_label_payload,
    candidate_payload_key,
    should_suppress_repeated_confirm,
)
from colmag_ros.scripts.dashboard_confirm_publisher import PreviewConfirmPublisher


REPO = Path(__file__).resolve().parents[1]
DASHBOARD = REPO / "colmag_ros" / "scripts" / "magnetic_trajectory_dashboard_node.py"
HELPER = REPO / "colmag_ros" / "src" / "colmag_ros" / "dashboard_confirm_publisher.py"


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
    node.integrated_confirm_enabled = integrated
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


class PreviewConfirmWiringTests(unittest.TestCase):
    def setUp(self):
        self.src = DASHBOARD.read_text()
        self.helper_src = HELPER.read_text()

    def test_preview_confirm_calls_publisher(self):
        # The preview C-confirm handler must invoke the publish path.
        self.assertIn("def _preview_confirm_candidate", self.src)
        self.assertIn("self._publish_preview_confirmed_label()", self.src)

    def test_business_logic_moved_to_helper_module(self):
        self.assertIn("from colmag_ros.dashboard_confirm_publisher import (", self.src)
        self.assertIn("publish_rank_one", self.helper_src)
        publish_method = self.src.split("def _publish_preview_confirmed_label", 1)[1].split(
            "def _top_preview_candidate_label", 1
        )[0]
        self.assertNotIn("build_confirmed_label_payload(payload, 1)", publish_method)
        self.assertNotIn("json.dumps(confirmed", publish_method)

    def test_helper_module_is_small(self):
        self.assertLess(HELPER.read_text().count("\n") + 1, 1000)

    def test_dashboard_has_no_task_command_publisher(self):
        # M95 must not wire the dashboard to /colmag/task_command.
        self.assertNotIn("Publisher(self.task_command_topic", self.src)

    def test_publish_path_reuses_shared_builder_and_dedup(self):
        self.assertIn("_build_confirmed_label_payload(payload, 1)", self.helper_src)
        self.assertIn("_should_suppress_repeated_confirm(last_confirm_key, payload, 1)", self.helper_src)


if __name__ == "__main__":
    unittest.main()

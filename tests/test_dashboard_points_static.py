"""Static/unit tests for ROS-free dashboard point helpers.

No ROS master, Tk GUI, Gazebo, serial board, controller manager, or real FR3
runtime is started.
"""

import ast
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
HELPER = REPO / "colmag_ros" / "src" / "colmag_ros" / "dashboard_points.py"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from colmag_ros import dashboard_points as points  # noqa: E402
from dashboard_harness import DashboardHarness  # noqa: E402


class DashboardPointHelperBehaviorTests(unittest.TestCase):
    def test_extract_xy_points_supports_existing_payload_shapes(self):
        self.assertEqual(points.extract_xy_points({}), [])
        self.assertEqual(
            points.extract_xy_points({
                "trajectory": [{"x": 0.1, "y": 0.2}, {"x": "0.3", "y": "0.4"}],
            }),
            [(0.1, 0.2), (0.3, 0.4)],
        )
        self.assertEqual(
            points.extract_xy_points({
                "points_2d": [{"position": {"x": 1, "y": 2}}],
            }),
            [(1.0, 2.0)],
        )
        self.assertEqual(
            points.extract_xy_points({"path": [{"point": {"x": 5, "y": 6}}]}),
            [(5.0, 6.0)],
        )
        self.assertEqual(
            points.extract_xy_points({"points": [{"x": 1}, {"y": 2}, {"x": 3, "y": "a"}]}),
            [],
        )

    def test_extract_single_point_preserves_existing_nested_order(self):
        self.assertEqual(points.extract_single_point({"x": 0.5, "y": -0.5}), (0.5, -0.5))
        self.assertEqual(points.extract_single_point({"point": {"x": 3, "y": 4}}), (3.0, 4.0))
        self.assertEqual(points.extract_single_point({"position": {"x": 1, "y": 2}}), (1.0, 2.0))
        self.assertIsNone(points.extract_single_point({}))
        self.assertIsNone(points.extract_single_point({"x": 1}))

    def test_clean_and_ocr_points_share_same_cleanup(self):
        trail = [(0, 0), None, (1, 1), None]
        self.assertEqual(points.clean_stroke_points(trail), [(0, 0), (1, 1)])
        self.assertEqual(points.build_ocr_stroke_points(trail), [(0, 0), (1, 1)])

    def test_flatten_strokes_preserves_order(self):
        self.assertEqual(
            points.flatten_strokes([[(0, 0), (1, 1)], [(2, 2)]]),
            [(0, 0), (1, 1), (2, 2)],
        )


def test_dashboard_flattens_strokes_through_helper(monkeypatch):
    harness = DashboardHarness(monkeypatch, demo_input_mode="ocr_canvas")
    node = harness.node
    harness.fire(node.ocr_button_widgets["B"]["canvas"], "<Button-1>")
    harness.draw_stroke((60, 60), (120, 60), (180, 60))
    harness.draw_stroke((60, 200), (120, 200))
    harness.run_jobs(node.ocr_auto_recognize_after_ms)

    [capture] = harness.published_json(node.config.symbol_capture_topic)
    assert [len(stroke) for stroke in node.all_strokes] == [3, 2]
    assert capture["points"] == [list(p) for p in points.flatten_strokes(node.all_strokes)]


class DashboardPointExtractionStaticTests(unittest.TestCase):
    def test_helper_has_no_ros_tk_or_execution_dependencies(self):
        text = HELPER.read_text()
        for forbidden in (
            "rospy",
            "tkinter",
            "Publisher(",
            "task_dispatcher_node.py",
            "fr3_gazebo_visible_task_bridge_node.py",
            "gazebo_task_executor.py",
            "/colmag/task_command",
            "FollowJointTrajectory",
        ):
            self.assertNotIn(forbidden, text)

    def test_helper_functions_are_small(self):
        tree = ast.parse(HELPER.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef):
                end = getattr(node, "end_lineno", node.lineno)
                self.assertLessEqual(end - node.lineno + 1, 40, node.name)


if __name__ == "__main__":
    unittest.main()

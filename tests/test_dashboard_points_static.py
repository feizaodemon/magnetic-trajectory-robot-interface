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
DASHBOARD = SCRIPTS / "magnetic_trajectory_dashboard_node.py"
HELPER = SCRIPTS / "dashboard_points.py"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import dashboard_points as points  # noqa: E402
import magnetic_trajectory_dashboard_node as dash  # noqa: E402


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


class DashboardPointExtractionStaticTests(unittest.TestCase):
    def test_dashboard_compatibility_wrappers_delegate_to_helper(self):
        self.assertEqual(dash.extract_xy_points({"points": [[1, 2]]}), [(1.0, 2.0)])
        self.assertEqual(dash.extract_single_point({"position": {"x": 1, "y": 2}}), (1.0, 2.0))
        self.assertEqual(dash.clean_stroke_points([(0, 0), None]), [(0, 0)])
        self.assertEqual(dash.build_ocr_stroke_points([(0, 0), None]), [(0, 0)])

        src = DASHBOARD.read_text()
        for expected in (
            "return _dashboard_points.extract_xy_points(payload)",
            "return _dashboard_points.extract_single_point(payload)",
            "return _dashboard_points.clean_stroke_points(trail)",
            "return _dashboard_points.build_ocr_stroke_points(trail)",
            "return _dashboard_points.flatten_strokes(self.all_strokes)",
        ):
            self.assertIn(expected, src)

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

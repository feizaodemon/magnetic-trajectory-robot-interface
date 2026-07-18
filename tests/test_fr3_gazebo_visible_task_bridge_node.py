import json
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

SCRIPTS = Path(__file__).resolve().parents[1] / "colmag_gazebo_stub" / "scripts"
sys.path.insert(0, str(SCRIPTS))

from fr3_gazebo_visible_task_bridge_node import (
    Fr3GazeboVisibleTaskBridgeNode,
    parse_task_command,
    PANDA_JOINTS
)


class TestFr3GazeboVisibleTaskBridgeNode(unittest.TestCase):
    def setUp(self):
        # Mock rospy to prevent ROS node initialization during pure unit tests
        self.patcher_rospy = patch("fr3_gazebo_visible_task_bridge_node.rospy")
        self.mock_rospy = self.patcher_rospy.start()

        self.patcher_actionlib = patch("fr3_gazebo_visible_task_bridge_node.actionlib")
        self.mock_actionlib = self.patcher_actionlib.start()

        # Mock publisher
        self.mock_pub = MagicMock()
        self.mock_rospy.Publisher.return_value = self.mock_pub

        # Provide default params for init
        def get_param_side_effect(param_name, default=None):
            if param_name == "~readiness_timeout_sec":
                return 0.5  # fast timeout for testing
            return default
        self.mock_rospy.get_param.side_effect = get_param_side_effect

        # Don't start the worker thread immediately in tests to avoid race conditions
        with patch("threading.Thread.start"):
            self.node = Fr3GazeboVisibleTaskBridgeNode()

        # Reset mock calls after init
        self.mock_pub.publish.reset_mock()

    def tearDown(self):
        self.patcher_rospy.stop()
        self.patcher_actionlib.stop()

    def _task_message(self, task):
        message = MagicMock()
        message.data = json.dumps({
            "task": task,
            "accepted": True,
            "controls_real_robot": False,
            "gazebo_only": True,
        })
        return message

    def _last_status(self):
        return json.loads(self.mock_pub.publish.call_args_list[-1][0][0].data)

    def test_parse_valid_task_command(self):
        data = '{"task": "HEXAGON_TRAJECTORY", "accepted": true}'
        payload, reason = parse_task_command(data)
        self.assertEqual(payload["task"], "HEXAGON_TRAJECTORY")
        self.assertEqual(reason, "")

        invalid_data = "not a json"
        payload, reason = parse_task_command(invalid_data)
        self.assertIsNone(payload)
        self.assertEqual(reason, "invalid_json")

    def test_reject_controls_real_robot(self):
        msg = MagicMock()
        msg.data = json.dumps({"task": "HEXAGON_TRAJECTORY", "controls_real_robot": True})
        self.node._handle_task_command(msg)

        self.mock_pub.publish.assert_called_once()
        published_json = json.loads(self.mock_pub.publish.call_args[0][0].data)
        self.assertEqual(published_json["status"], "rejected")
        self.assertEqual(published_json["reason"], "real_robot_not_supported_in_gazebo_bridge")

    def test_reject_missing_gazebo_only(self):
        msg = MagicMock()
        msg.data = json.dumps({"task": "HEXAGON_TRAJECTORY", "controls_real_robot": False})
        self.node._handle_task_command(msg)

        self.mock_pub.publish.assert_called_once()
        published_json = json.loads(self.mock_pub.publish.call_args[0][0].data)
        self.assertEqual(published_json["status"], "rejected")
        self.assertEqual(published_json["reason"], "gazebo_only_flag_missing_or_false")

    def test_task_is_queued_before_joint_state_ready(self):
        msg = MagicMock()
        msg.data = json.dumps({"task": "HEXAGON_TRAJECTORY", "gazebo_only": True})
        self.node._handle_task_command(msg)

        calls = self.mock_pub.publish.call_args_list
        last_call_json = json.loads(calls[-1][0][0].data)

        self.assertEqual(last_call_json["phase"], "queued_until_ready")
        self.assertEqual(self.node.queued_task, "HEXAGON_TRAJECTORY")

    def test_no_crash_when_joint_positions_are_none_and_not_ready(self):
        self.node.latest_joint_positions = None
        self.node.server_ready = False
        self.node.queued_task = "HEXAGON_TRAJECTORY"
        self.node.queued_time = time.time()

        # Test a single pass of the worker loop logic
        # It shouldn't crash
        with patch("fr3_gazebo_visible_task_bridge_node.rospy.is_shutdown", side_effect=[False, True]):
            self.node._worker_loop()

        # Still queued
        self.assertEqual(self.node.queued_task, "HEXAGON_TRAJECTORY")

    @patch("fr3_gazebo_visible_task_bridge_node.rospy.sleep")
    def test_queued_task_executes_once_joint_state_available(self, mock_sleep):
        self.node.latest_joint_positions = [0.0] * 7
        self.node.latest_joint_state_wall_time = time.time()
        self.node.server_ready = True
        self.node.client.wait_for_server.return_value = True
        self.node.client.wait_for_result.return_value = True
        self.node.client.get_state.return_value = 3  # ACTION_SUCCEEDED

        self.node.queued_task = "HEXAGON_TRAJECTORY"
        self.node.queued_time = time.time()

        with patch("fr3_gazebo_visible_task_bridge_node.rospy.is_shutdown", side_effect=[False, True]):
            self.node._worker_loop()

        self.assertIsNone(self.node.queued_task)  # Task was consumed
        self.node.client.send_goal.assert_called_once()

        calls = self.mock_pub.publish.call_args_list
        last_call_json = json.loads(calls[-1][0][0].data)
        self.assertEqual(last_call_json["phase"], "succeeded")

    def test_readiness_timeout_publishes_failed(self):
        self.node.queued_task = "HEXAGON_TRAJECTORY"
        self.node.queued_time = time.time() - 2.0  # past the 0.5s timeout

        with patch("fr3_gazebo_visible_task_bridge_node.rospy.is_shutdown", side_effect=[False, True]):
            self.node._worker_loop()

        self.assertIsNone(self.node.queued_task)  # Consumed by timeout
        calls = self.mock_pub.publish.call_args_list
        last_call_json = json.loads(calls[-1][0][0].data)
        self.assertEqual(last_call_json["status"], "failed")
        self.assertEqual(last_call_json["reason"], "readiness_timeout_exceeded")

    def test_stop_with_active_goal_cancels_once_and_succeeds_no_motion(self):
        self.node.goal_active = True
        self.node.client.get_state.return_value = 1
        self.node._handle_task_command(self._task_message("STOP_OR_CANCEL"))

        self.node.client.cancel_goal.assert_called_once()
        self.assertFalse(self.node.goal_active)
        self.assertEqual(self._last_status()["status"], "succeeded")
        self.assertEqual(self._last_status()["phase"], "succeeded")
        self.assertTrue(self._last_status()["no_motion"])

    def test_stop_with_pending_goal_cancels_once(self):
        self.node.goal_active = True
        self.node.client.get_state.return_value = 0
        self.node._handle_task_command(self._task_message("STOP_OR_CANCEL"))

        self.node.client.cancel_goal.assert_called_once()
        self.assertFalse(self.node.goal_active)

    @patch("fr3_gazebo_visible_task_bridge_node.rospy.sleep")
    def test_stop_after_succeeded_goal_does_not_cancel_terminal_goal(self, mock_sleep):
        self.node.client.wait_for_result.return_value = True
        self.node.client.get_state.return_value = 3
        self.node._execute_task("HEXAGON_TRAJECTORY", [0.0] * 7, {})
        self.node._handle_task_command(self._task_message("STOP_OR_CANCEL"))

        self.node.client.cancel_goal.assert_not_called()
        self.assertEqual(self._last_status()["task"], "STOP_OR_CANCEL")
        self.assertEqual(self._last_status()["status"], "succeeded")
        self.assertTrue(self._last_status()["no_motion"])

    def test_stop_without_any_goal_does_not_cancel_and_succeeds_no_motion(self):
        self.node._handle_task_command(self._task_message("STOP_OR_CANCEL"))

        self.node.client.cancel_goal.assert_not_called()
        self.assertEqual(self._last_status()["status"], "succeeded")
        self.assertTrue(self._last_status()["no_motion"])

    def test_repeated_stop_only_claims_active_goal_once(self):
        self.node.goal_active = True
        self.node.client.get_state.return_value = 1
        message = self._task_message("STOP_OR_CANCEL")
        self.node._handle_task_command(message)
        self.node._handle_task_command(message)

        self.node.client.cancel_goal.assert_called_once()

    @patch("fr3_gazebo_visible_task_bridge_node.rospy.sleep")
    def test_move_left_and_hexagon_success_behavior_is_unchanged(self, mock_sleep):
        self.node.client.wait_for_result.return_value = True
        self.node.client.get_state.return_value = 3

        for task in ("MOVE_LEFT", "HEXAGON_TRAJECTORY"):
            with self.subTest(task=task):
                self.mock_pub.publish.reset_mock()
                self.node._execute_task(task, [0.0] * 7, {})
                self.assertEqual(self._last_status()["task"], task)
                self.assertEqual(self._last_status()["status"], "succeeded")

    def test_output_safety_flags(self):
        self.node._publish_status("succeeded", "HEXAGON_TRAJECTORY", "succeeded")

        self.mock_pub.publish.assert_called_once()
        published_json = json.loads(self.mock_pub.publish.call_args[0][0].data)

        self.assertTrue(published_json["gazebo_only"])
        self.assertFalse(published_json["controls_real_robot"])
        self.assertTrue(published_json["placeholder_only"])
        self.assertEqual(published_json["robot_family"], "FR3_GAZEBO_PLACEHOLDER")

    def test_trajectory_helper_builds_7_joint_points(self):
        current_pos = [0.0] * 7
        waypoints = [[0.1] * 7, [0.2] * 7]
        goal = self.node._build_goal(current_pos, waypoints)

        self.assertEqual(len(goal.trajectory.joint_names), 7)
        self.assertEqual(goal.trajectory.joint_names, PANDA_JOINTS)

        self.assertEqual(len(goal.trajectory.points), 3)
        for point in goal.trajectory.points:
            self.assertEqual(len(point.positions), 7)

if __name__ == '__main__':
    unittest.main()

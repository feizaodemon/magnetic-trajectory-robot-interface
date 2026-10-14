"""ROS-free rules of the Gazebo TELEOP gripper bridge."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from colmag_gazebo_stub import gripper_bridge

REPO = Path(__file__).resolve().parents[1]
STUB = REPO / "colmag_gazebo_stub"
TELEOP_LAUNCH = REPO / "colmag_ros" / "launch" / "colmag_keyboard_gazebo_teleop.launch"


def _payload(**overrides):
    payload = {"command": "close", "sequence_id": 3, "timestamp": 10.0}
    payload.update(overrides)
    return json.dumps(payload)


def test_parse_command_accepts_open_and_close_only():
    assert gripper_bridge.parse_command(_payload()) == ("close", 3, 10.0)
    assert gripper_bridge.parse_command(_payload(command=" OPEN "))[0] == "open"
    for bad in (_payload(command="grip"), _payload(sequence_id="x"),
                _payload(timestamp=float("nan")), "[]", "not json"):
        with pytest.raises(ValueError):
            gripper_bridge.parse_command(bad)


def test_gate_admits_only_fresh_new_commands_in_teleop():
    gate = gripper_bridge.CommandGate(max_age_sec=1.0)
    assert gate.admit(1, 10.0, "TASK", True, 10.0) == "teleop_not_owner"
    assert gate.admit(1, 10.0, "TELEOP", False, 10.0) == "teleop_not_owner"
    assert gate.admit(1, 8.5, "TELEOP", True, 10.0) == "stale_gripper_command"
    assert gate.admit(1, 10.0, "TELEOP", True, 10.2) == ""
    assert gate.admit(1, 10.3, "TELEOP", True, 10.3) == "non_monotonic_gripper_command"
    assert gate.admit(2, 10.3, "teleop", True, 10.3) == ""


def test_finger_width_and_settled_state():
    names = ["panda_finger_joint1", "panda_finger_joint2"]
    assert gripper_bridge.finger_width(names, [0.02, 0.025]) == pytest.approx(0.045)
    assert gripper_bridge.finger_width(["panda_joint1"], [0.1]) is None
    settled = gripper_bridge.settled_state
    assert settled("open", True, 0.08, 0.005) == gripper_bridge.OPENED
    assert settled("open", False, 0.03, 0.005) == gripper_bridge.FAILED
    assert settled("close", True, 0.048, 0.005) == gripper_bridge.HOLDING
    assert settled("close", True, 0.001, 0.005) == gripper_bridge.CLOSED
    assert settled("close", False, 0.03, 0.005) == gripper_bridge.FAILED
    assert gripper_bridge.state_payload("HOLDING", "", "close", 0.04812) == {
        "state": "HOLDING", "reason": "", "command": "close", "width_m": 0.0481}


def test_launch_starts_the_bridge_and_spawns_objects_only_on_request():
    root = ET.parse(TELEOP_LAUNCH).getroot()
    args = {node.attrib["name"]: node.attrib.get("default") for node in root.findall("arg")}
    assert args["start_gripper_bridge"] == "true"
    assert args["spawn_pick_objects"] == "false"
    assert args["gripper_command_topic"] == "/colmag/teleop/gripper_command"
    bridge = next(node for node in root.iter("node")
                  if node.attrib.get("type") == "gazebo_gripper_bridge_node.py")
    assert bridge.attrib["if"] == "$(arg start_gripper_bridge)"
    spawn = root.find("group")
    assert spawn.attrib["if"] == "$(arg spawn_pick_objects)"
    for node in spawn.iter("node"):
        model = node.attrib["args"].split("models/")[1].split()[0]
        assert (STUB / "models" / model).is_file()


def test_bridge_node_is_installed_and_gazebo_only():
    cmake = (STUB / "CMakeLists.txt").read_text()
    assert "scripts/gazebo_gripper_bridge_node.py" in cmake
    assert "install(DIRECTORY models" in cmake
    source = (STUB / "scripts" / "gazebo_gripper_bridge_node.py").read_text()
    assert "/franka_gripper" in source
    for forbidden in ("robot_ip", "franka_control", "libfranka"):
        assert forbidden not in source

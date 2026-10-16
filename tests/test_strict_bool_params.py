"""Boolean node params are parsed strictly.

``bool(rospy.get_param(...))`` made the string "false" true. The keyboard
teleop nodes (``~require_subscriber``) and the serial packet publisher
(``~debug``) now use ``param_utils.strict_bool_param``: same param names and
defaults, "false" is false, and a value that is neither true nor false stops
the node instead of being guessed. No ROS master or node is started; rospy
and std_msgs are replaced with fakes.
"""

import importlib.util
import sys
import types
from pathlib import Path

import pytest

from colmag_ros.param_utils import strict_bool_param

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"


@pytest.mark.parametrize("value", [True, 1, "true", "True", " TRUE ", "yes", "on", "1"])
def test_true_values(value):
    assert strict_bool_param(value, "~p") is True


@pytest.mark.parametrize("value", [False, 0, "false", "False", " false\n", "no", "off", "0"])
def test_false_values(value):
    assert strict_bool_param(value, "~p") is False


@pytest.mark.parametrize("value", ["", "ture", "none", 2, -1, 0.0, 1.0, None, [], {}])
def test_unclear_values_raise_and_name_the_param(value):
    with pytest.raises(ValueError, match="~require_subscriber"):
        strict_bool_param(value, "~require_subscriber")


class FakePublisher:
    def __init__(self, topic, msg_class, queue_size=None, latch=False):
        self.topic = topic

    def get_num_connections(self):
        return 0


def _fake_ros(monkeypatch, params):
    rospy = types.ModuleType("rospy")
    rospy.get_param = lambda name, default=None: params.get(name, default)
    rospy.Publisher = FakePublisher
    rospy.loginfo = rospy.logwarn = lambda *args, **kwargs: None
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg.String = type("String", (), {"__init__": lambda self, data="": None})
    monkeypatch.setitem(sys.modules, "rospy", rospy)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs.msg)


def _load(name):
    spec = importlib.util.spec_from_file_location("strict_bool_" + name, SCRIPTS / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


KEYBOARD_NODES = [
    ("keyboard_teleop_node", "KeyboardTeleopNode"),
    ("keyboard_cartesian_teleop_node", "KeyboardCartesianTeleopNode"),
]


@pytest.mark.parametrize("script,cls", KEYBOARD_NODES)
@pytest.mark.parametrize("params,expected", [
    ({}, True),  # default unchanged
    ({"~require_subscriber": True}, True),
    ({"~require_subscriber": False}, False),
    ({"~require_subscriber": "false"}, False),  # was True with bool()
    ({"~require_subscriber": "true"}, True),
])
def test_keyboard_nodes_read_require_subscriber(monkeypatch, script, cls, params, expected):
    _fake_ros(monkeypatch, params)
    node = getattr(_load(script), cls)()
    assert node.require_subscriber is expected


@pytest.mark.parametrize("script,cls", KEYBOARD_NODES)
def test_keyboard_nodes_reject_an_unclear_require_subscriber(monkeypatch, script, cls):
    _fake_ros(monkeypatch, {"~require_subscriber": "maybe"})
    with pytest.raises(ValueError, match="~require_subscriber"):
        getattr(_load(script), cls)()


@pytest.mark.parametrize("params,expected", [
    ({}, False),  # default unchanged
    ({"~debug": True}, True),
    ({"~debug": "false"}, False),  # was True with bool()
    ({"~debug": "true"}, True),
])
def test_serial_publisher_reads_debug(monkeypatch, params, expected):
    pytest.importorskip("serial")
    _fake_ros(monkeypatch, dict(params, **{"~project_src_path": str(REPO / "src")}))
    node = _load("serial_packet_publisher_node").SerialPacketPublisherNode()
    assert node.debug is expected


def test_serial_publisher_rejects_an_unclear_debug(monkeypatch):
    _fake_ros(monkeypatch, {"~debug": "verbose", "~project_src_path": str(REPO / "src")})
    with pytest.raises(ValueError, match="~debug"):
        _load("serial_packet_publisher_node").SerialPacketPublisherNode()

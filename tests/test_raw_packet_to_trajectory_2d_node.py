import json
from std_msgs.msg import String

class MockPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)

class MockTime:
    def get_time(self):
        return 1234.5

# We must mock rospy before importing our node
import sys
from unittest import mock
import pytest

mock_rospy = mock.MagicMock()
mock_rospy.get_time = MockTime().get_time
mock_rospy.get_param.side_effect = lambda param_name, default=None: default
sys.modules["rospy"] = mock_rospy
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / "colmag_ros" / "scripts"))
from raw_packet_to_trajectory_2d_node import RawPacketToTrajectory2DNode


def test_raw_packet_parsing():
    node = RawPacketToTrajectory2DNode()
    pub = MockPublisher()
    node.publisher = pub

    # Test valid input
    raw_packet = {
        "timestamp": 1000.0,
        "magnetic_readings": [],
        "tracking_outputs": [0.14, -0.03, 0.0, 0.0, 0.0, 1.0]
    }

    msg = String(data=json.dumps(raw_packet))
    node._handle_packet(msg)

    assert len(pub.published) == 1
    out = json.loads(pub.published[0].data)
    assert out["timestamp"] == 1000.0
    assert out["x"] == 0.14
    assert out["y"] == -0.03
    assert out["valid"] is True
    assert out["source"] == "raw_packet_bridge_offline"
    assert out["source_topic"] == "/colmag/raw_packet"
    assert out["phase"] == "raw_packet_bridge"
    assert out["sequence_id"] == 1
    assert out["sample_index"] == 0


def test_malformed_packet_handled_safely():
    node = RawPacketToTrajectory2DNode()
    pub = MockPublisher()
    node.publisher = pub

    # Bad JSON
    node._handle_packet(String(data="not json"))
    assert len(pub.published) == 0

    # Not an object
    node._handle_packet(String(data="[]"))
    assert len(pub.published) == 0

    # Missing tracking_outputs
    node._handle_packet(String(data='{"timestamp": 0.0}'))
    assert len(pub.published) == 0

    # Invalid tracking_outputs types
    node._handle_packet(String(data='{"tracking_outputs": ["a", "b"]}'))
    assert len(pub.published) == 0


def test_invalid_tracking_marked_invalid():
    node = RawPacketToTrajectory2DNode()
    pub = MockPublisher()
    node.publisher = pub

    # tracking_outputs where valid is 0.0
    raw_packet = {
        "timestamp": 2000.0,
        "tracking_outputs": [0.1, 0.2, 0.0, 0.0, 0.0, 0.0]
    }
    msg = String(data=json.dumps(raw_packet))
    node._handle_packet(msg)

    assert len(pub.published) == 1
    out = json.loads(pub.published[0].data)
    assert out["valid"] is False


@pytest.mark.parametrize(
    ("tracking_mz", "interaction_engaged"),
    [(-1.0, False), (0.0, False), (0.001, True)],
)
def test_valid_field_preserves_orientation_interaction_clutch_contract(
    tracking_mz,
    interaction_engaged,
):
    node = RawPacketToTrajectory2DNode()
    pub = MockPublisher()
    node.publisher = pub

    raw_packet = {
        "timestamp": 3000.0,
        "tracking_outputs": [0.1, 0.2, 0.0, 0.0, 0.0, tracking_mz],
    }
    node._handle_packet(String(data=json.dumps(raw_packet)))

    assert json.loads(pub.published[0].data)["valid"] is interaction_engaged

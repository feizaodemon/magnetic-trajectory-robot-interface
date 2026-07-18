#!/usr/bin/env python3
import json
import math
import sys
import time
import uuid
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros import m104c4_execution_semantics

_C4_CONTRACT = m104c4_execution_semantics.validated_execution_contract()
C4_MAPPING_NAME = _C4_CONTRACT["mapping_name"]
C4_LABEL_TO_TASK = _C4_CONTRACT["label_to_task"]
C4_SAFE_GAZEBO_TASKS = set(_C4_CONTRACT["safe_tasks"])
DEFAULT_LABEL_TO_TASK = dict(C4_LABEL_TO_TASK)

SAFE_GAZEBO_DEMO_TASKS = set(C4_SAFE_GAZEBO_TASKS)

REVIEW_TASKS = {
    "PICK_PLACE",
    "SORT_OBJECTS",
    "STACK_BLOCKS",
    "PRESS_BUTTON",
    "OPEN_DRAWER",
    "AVOID_OBSTACLE",
    "INSPECT_OBJECT",
}

CRITICAL_TASKS = {
    "STOP",
    "CLEAR",
    "COMPLIANT_TOUCH",
    "COMPLIANT_CONTROL",
}

STOP_CLEAR_TASKS = {"STOP", "CLEAR"}


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def normalize_allowed_task_set(value):
    name = str(value or "").strip().lower()
    if name in ("", "all", "all_legacy", "legacy"):
        return "all_legacy"
    if name == "safe_gazebo_demo":
        return "safe_gazebo_demo"
    return "safe_gazebo_demo"


def task_safety_class(task):
    task = str(task or "").strip().upper()
    if task in CRITICAL_TASKS:
        return "CRITICAL"
    if task in REVIEW_TASKS:
        return "REVIEW"
    if task in SAFE_GAZEBO_DEMO_TASKS:
        return "SAFE"
    return "REVIEW"


def should_publish_task_command(command, allowed_task_set="all_legacy", allow_critical_tasks=True):
    if not isinstance(command, dict):
        return False, "command_not_object"

    task_set = normalize_allowed_task_set(allowed_task_set)
    task = str(command.get("task", "NO_OP")).strip().upper() or "NO_OP"
    accepted = bool_param(command.get("accepted", False))
    safety_class = str(command.get("safety_class") or task_safety_class(task)).strip().upper()

    if not accepted:
        if task_set == "all_legacy":
            return True, ""
        return False, command.get("reason") or "task_not_accepted"

    if task_set == "safe_gazebo_demo" and task in STOP_CLEAR_TASKS:
        return False, "stop_clear_requires_separate_path"

    if safety_class == "CRITICAL" and not bool_param(allow_critical_tasks):
        return False, "critical_task_blocked"

    if task_set == "safe_gazebo_demo" and task not in SAFE_GAZEBO_DEMO_TASKS:
        return False, "task_not_in_allowed_set"

    return True, ""


def finite_float(value, default=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def parse_mapping(mapping_text):
    if isinstance(mapping_text, dict):
        return {str(k).upper(): str(v) for k, v in mapping_text.items()}
    try:
        parsed = json.loads(mapping_text)
    except Exception:
        return dict(DEFAULT_LABEL_TO_TASK)
    if not isinstance(parsed, dict):
        return dict(DEFAULT_LABEL_TO_TASK)
    return {str(k).upper(): str(v) for k, v in parsed.items()}


def label_to_task_mapping_for_name(mapping_name):
    name = str(mapping_name or "").strip().lower()
    if name == C4_MAPPING_NAME:
        return dict(C4_LABEL_TO_TASK)
    return dict(DEFAULT_LABEL_TO_TASK)


def build_task_command(
    confirmed_payload,
    label_to_task=None,
    min_confidence=0.0,
    gazebo_only=True,
    dispatcher_mapping_name="m104c4_8symbol_gazebo",
    command_id=None,
    issued_at=None,
    source_id="task_dispatcher_node",
    schema_version=1,
    target_adapter_session_id="",
):
    label_to_task = label_to_task or label_to_task_mapping_for_name(dispatcher_mapping_name)
    label = str(confirmed_payload.get("label", "")).strip().upper()
    confirmed = bool_param(confirmed_payload.get("confirmed", False))
    confidence = finite_float(confirmed_payload.get("confidence"), 0.0)

    task = "NO_OP"
    accepted = False
    reason = ""

    if not confirmed:
        reason = "confirmed_label_false"
    elif confidence < min_confidence:
        reason = "confidence_below_threshold"
    else:
        task = label_to_task.get(label, "NO_OP")
        accepted = task != "NO_OP"
        if not accepted:
            reason = "unknown_label"

    issued_at = time.time() if issued_at is None else finite_float(issued_at, 0.0)
    command_id = str(command_id or uuid.uuid4().hex)
    return {
        "timestamp": issued_at,
        "command_id": command_id,
        "issued_at": issued_at,
        "source_id": str(source_id),
        "schema_version": int(schema_version),
        "target_adapter_session_id": str(target_adapter_session_id or ""),
        "task": task,
        "source_label": label,
        "source_confidence": confidence,
        "confirmed": confirmed,
        "accepted": accepted,
        "reason": reason,
        "mapping": dispatcher_mapping_name,
        "safety_class": task_safety_class(task),
        "gazebo_only": bool(gazebo_only),
        "controls_real_robot": False,
        "upstream_controls_real_robot": False,
        "source_topic": "/colmag/confirmed_label",
        "sample_id": confirmed_payload.get("sample_id", ""),
        "sequence_id": confirmed_payload.get("sequence_id", 0),
    }


class TaskDispatcherNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.input_topic = rospy.get_param("~confirmed_label_topic", "/colmag/confirmed_label")
        self.output_topic = rospy.get_param("~task_command_topic", "/colmag/task_command")
        self.adapter_session_topic = rospy.get_param(
            "~adapter_session_topic", "/colmag/fr3_adapter_session"
        )
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.0))
        self.gazebo_only = bool_param(rospy.get_param("~gazebo_only", True))
        self.publish_once = bool_param(rospy.get_param("~publish_once", False))
        self.dispatcher_mapping_name = rospy.get_param("~dispatcher_mapping_name", "m104c4_8symbol_gazebo")
        self.allowed_task_set = normalize_allowed_task_set(rospy.get_param("~allowed_task_set", "all_legacy"))
        self.allow_critical_tasks = bool_param(rospy.get_param("~allow_critical_tasks", True))
        self.source_id = str(rospy.get_param("~source_id", "task_dispatcher_node"))
        self.schema_version = int(rospy.get_param("~schema_version", 1))
        self.has_published = False
        self.adapter_session_id = ""
        label_to_task_json = rospy.get_param("~label_to_task_json", "")
        if label_to_task_json:
            self.label_to_task = parse_mapping(label_to_task_json)
        else:
            self.label_to_task = label_to_task_mapping_for_name(self.dispatcher_mapping_name)
        self.publisher = rospy.Publisher(self.output_topic, String, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber(self.input_topic, String, self._handle_confirmed_label, queue_size=10)
        self.adapter_session_subscriber = rospy.Subscriber(
            self.adapter_session_topic, String, self._handle_adapter_session, queue_size=1
        )
        rospy.loginfo("task_dispatcher_node started")
        rospy.loginfo("input_topic=%s output_topic=%s", self.input_topic, self.output_topic)
        rospy.loginfo("adapter_session_topic=%s", self.adapter_session_topic)
        rospy.loginfo("label_to_task=%s", self.label_to_task)
        rospy.loginfo("allowed_task_set=%s allow_critical_tasks=%s",
                      self.allowed_task_set, self.allow_critical_tasks)

    def _handle_adapter_session(self, message):
        self.adapter_session_id = str(message.data or "").strip()

    def _handle_confirmed_label(self, message):
        if self.publish_once and self.has_published:
            return

        try:
            payload = json.loads(message.data)
        except ValueError as exc:
            self.rospy.logwarn("failed to parse confirmed_label JSON: %r", exc)
            return
        if not isinstance(payload, dict):
            self.rospy.logwarn("confirmed_label payload is not a JSON object")
            return

        command = build_task_command(
            payload,
            label_to_task=self.label_to_task,
            min_confidence=self.min_confidence,
            gazebo_only=self.gazebo_only,
            dispatcher_mapping_name=self.dispatcher_mapping_name,
            command_id=uuid.uuid4().hex,
            issued_at=self.rospy.Time.now().to_sec(),
            source_id=self.source_id,
            schema_version=self.schema_version,
            target_adapter_session_id=getattr(self, "adapter_session_id", ""),
        )
        should_publish, safety_reason = should_publish_task_command(
            command,
            allowed_task_set=self.allowed_task_set,
            allow_critical_tasks=self.allow_critical_tasks,
        )
        if not should_publish:
            self.rospy.logwarn("task_command rejected by safety gate: task=%s allowed_task_set=%s reason=%s",
                               command.get("task", "NO_OP"), self.allowed_task_set, safety_reason)
            return

        msg = self.String(data=json.dumps(command, sort_keys=True))
        self.publisher.publish(msg)
        self.rospy.loginfo("task_command=%s", msg.data)

        if command["accepted"]:
            self.has_published = True


def main():
    import rospy

    rospy.init_node("task_dispatcher_node")
    TaskDispatcherNode()
    rospy.spin()


if __name__ == "__main__":
    main()

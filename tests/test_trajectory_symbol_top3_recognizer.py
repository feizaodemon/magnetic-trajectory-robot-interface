import importlib.util
import json
import sys
from pathlib import Path


def load_recognizer_module():
    path = (
        Path(__file__).resolve().parents[1]
        / "colmag_ros"
        / "scripts"
        / "trajectory_symbol_top3_recognizer_node.py"
    )
    spec = importlib.util.spec_from_file_location("trajectory_symbol_top3_recognizer_node", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


recognizer = load_recognizer_module()


class CallbackString:
    def __init__(self, data=""):
        self.data = data


class CallbackPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CallbackLogger:
    def __init__(self):
        self.warnings = []

    def logwarn(self, *args):
        self.warnings.append(args)

    def loginfo(self, *args):
        pass

    def logdebug(self, *args):
        pass


def make_callback_node(backend="dtw_template_bank"):
    node = recognizer.TrajectorySymbolTop3RecognizerNode.__new__(
        recognizer.TrajectorySymbolTop3RecognizerNode
    )
    node.rospy = CallbackLogger()
    node.String = CallbackString
    node.candidates_pub = CallbackPublisher()
    node.label_pub = CallbackPublisher()
    node.backend = backend
    node.labels = ("1", "2", "3")
    node.resample_length = 64
    node.min_confidence = 0.70
    node.min_margin = 0.20
    return node


def test_catkin_wrapper_exec_adds_package_source_to_import_path():
    path = (
        Path(__file__).resolve().parents[1]
        / "colmag_ros"
        / "scripts"
        / "trajectory_symbol_top3_recognizer_node.py"
    )
    package_src = str(path.parents[1] / "src")
    helper_modules = (
        "colmag_ros.recognition_uncertainty",
        "colmag_ros.trajectory_strokes",
    )
    saved_path = list(sys.path)
    saved_modules = {name: sys.modules.get(name) for name in helper_modules}

    try:
        sys.path[:] = [entry for entry in sys.path if entry != package_src]
        for name in helper_modules:
            sys.modules.pop(name, None)

        context = {
            "__file__": str(path),
            "__name__": "catkin_devel_wrapper_exec_test",
            "__package__": None,
        }
        exec(compile(path.read_text(), str(path), "exec"), context)

        assert str(context["_PACKAGE_SRC"]) == package_src
        assert sys.path[0] == package_src
        assert callable(context["apply_uncertainty_gate"])
        assert callable(context["flatten_strokes"])
    finally:
        sys.path[:] = saved_path
        for name, module in saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


def test_recognizer_returns_top3_json_payload_for_digit_two():
    points = [
        [-0.65, 0.65],
        [-0.15, 0.90],
        [0.55, 0.65],
        [0.35, 0.15],
        [-0.45, -0.45],
        [-0.65, -0.80],
        [0.65, -0.80],
    ]

    candidates = recognizer.recognize_top3(points)

    assert len(candidates) == 3
    assert candidates[0]["label"] == "2"
    assert candidates[0]["rank"] == 1
    assert candidates[1]["rank"] == 2
    assert candidates[2]["rank"] == 3


def test_callback_invalid_json_and_non_object_keep_invalid_payload_schema(monkeypatch):
    monkeypatch.setattr(recognizer.time, "time", lambda: 123.0)
    for raw, expected_reason in (("{", "Expecting property name"), ("[]", "payload_not_object")):
        node = make_callback_node()
        node._handle_capture(CallbackString(raw))
        payload = json.loads(node.candidates_pub.messages[-1].data)
        assert payload["timestamp"] == 123.0
        assert payload["reason"] == "invalid_payload"
        assert payload["backend"] == "dtw_template_bank"
        assert payload["sample_id"] == ""
        assert payload["sequence_id"] == 0
        assert payload["candidates"] == []
        assert expected_reason in payload["invalid_payload_reason"]
        assert node.label_pub.messages[-1].data == node.candidates_pub.messages[-1].data


def test_builtin_templates_cover_current_dtw_labels():
    labels = ("1", "2", "3", "V", "O", "X", "A", "C")

    templates = recognizer.build_builtin_templates(labels)

    assert {template["label"] for template in templates} == set(labels)


def test_default_dtw_helpers_need_no_optional_recognizer_dependency():
    points = [[0.0, 0.0], [1.0, 1.0]]
    candidates = recognizer.recognize_top3(points)

    assert len(candidates) > 0


def test_recognizer_node_does_not_publish_unauthorized_topics():
    path = Path(__file__).resolve().parents[1] / "colmag_ros" / "scripts" / "trajectory_symbol_top3_recognizer_node.py"
    content = path.read_text()

    # Must not publish to these topics to respect safety boundaries
    assert "/colmag/confirmed_label" not in content
    assert "/colmag/task_command" not in content
    assert "/colmag/robot_command" not in content

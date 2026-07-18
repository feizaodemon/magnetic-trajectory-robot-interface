#!/usr/bin/env python3
import json
import math
import sys
import time
from pathlib import Path


_PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
if _PACKAGE_SRC.is_dir() and str(_PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_SRC))

from colmag_ros.recognition_uncertainty import apply_uncertainty_gate
from colmag_ros.trajectory_strokes import flatten_strokes, normalize_payload_strokes, stroke_metadata
from colmag_ros.dtw_bank_features import (
    dtw_distance as _dtw_distance,
    finite_float as _finite_float,
    normalize_points as _normalize_points,
    resample_by_arclength as _resample_by_arclength,
)

DEFAULT_LABELS = ("1", "2", "3", "V", "O", "X", "A", "C")

def finite_float(value, default=0.0):
    return _finite_float(value, default)

def resample_by_arclength(points, n=64):
    return _resample_by_arclength(points, n)

def normalize_points(points, resample_length=64):
    return _normalize_points(points, resample_length)

def dtw_distance(a, b):
    return _dtw_distance(a, b)

def _line(points):
    return [(float(x), float(y)) for x, y in points]

BUILTIN_TEMPLATE_POINTS = {
    "A": _line([(-0.8, -0.8), (-0.45, 0.1), (0.0, 0.9), (0.45, 0.1), (0.8, -0.8), (0.35, -0.05), (-0.35, -0.05)]),
    "2": _line([(-0.65, 0.65), (-0.15, 0.9), (0.55, 0.65), (0.35, 0.15), (-0.45, -0.45), (-0.65, -0.8), (0.65, -0.8)]),
    "C": _line([(0.65, 0.65), (0.15, 0.9), (-0.55, 0.65), (-0.8, 0.0), (-0.55, -0.65), (0.15, -0.9), (0.65, -0.65)]),
    "X": _line([(-0.75, 0.75), (0.75, -0.75), (0.0, 0.0), (0.75, 0.75), (-0.75, -0.75)]),
    "1": _line([(-0.25, 0.55), (0.0, 0.85), (0.0, -0.8)]),
    "3": _line([(-0.55, 0.75), (0.45, 0.75), (0.0, 0.05), (0.5, -0.1), (0.35, -0.75), (-0.55, -0.7)]),
    "V": _line([(-0.75, 0.75), (0.0, -0.85), (0.75, 0.75)]),
    "O": _line([(0.0, 0.9), (0.65, 0.65), (0.9, 0.0), (0.65, -0.65), (0.0, -0.9), (-0.65, -0.65), (-0.9, 0.0), (-0.65, 0.65), (0.0, 0.9)]),
}

def build_builtin_templates(labels=DEFAULT_LABELS, resample_length=64):
    templates = []
    for label in labels:
        if label not in BUILTIN_TEMPLATE_POINTS:
            continue
        templates.append(
            {
                "label": label,
                "points": normalize_points(BUILTIN_TEMPLATE_POINTS[label], resample_length),
                "template_id": "builtin_%s" % label,
            }
        )
    return templates

def confidence_from_distances(distance, worst_distance):
    if not math.isfinite(distance):
        return 0.0
    if not math.isfinite(worst_distance) or worst_distance <= 1e-9:
        return 1.0
    return max(0.0, min(1.0, 1.0 - distance / worst_distance))

def recognize_top3(points, labels=DEFAULT_LABELS, resample_length=64):
    normalized = normalize_points(points, resample_length)
    templates = build_builtin_templates(labels, resample_length)
    distances = []
    for template in templates:
        distances.append((dtw_distance(normalized, template["points"]), template))
    distances.sort(key=lambda item: item[0])

    top = distances[:3]
    worst_distance = distances[-1][0] if distances else float("inf")
    candidates = []
    for rank, (distance, template) in enumerate(top, start=1):
        candidates.append(
            {
                "rank": rank,
                "label": template["label"],
                "confidence": confidence_from_distances(distance, worst_distance),
                "distance": distance,
            }
        )
    return candidates


def feature_mode_for_backend(backend):
    return "trajectory_dtw"


def ocr_result_source_for_backend(backend, candidates):
    return "not_used"

class TrajectorySymbolTop3RecognizerNode:
    def __init__(self):
        import rospy
        from std_msgs.msg import String

        self.rospy = rospy
        self.String = String
        self.input_topic = rospy.get_param("~symbol_capture_topic", "/colmag/symbol_capture")
        self.candidates_topic = rospy.get_param("~symbol_candidates_topic", "/colmag/symbol_candidates")
        self.recognized_label_topic = rospy.get_param("~recognized_label_topic", "/colmag/recognized_label")
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.70))
        self.min_margin = float(rospy.get_param("~min_margin", 0.20))
        self.resample_length = int(rospy.get_param("~resample_length", 64))
        self.labels = tuple(
            label.strip().upper()
            for label in str(rospy.get_param("~label_set", ",".join(DEFAULT_LABELS))).split(",")
            if label.strip()
        )

        # The DTW template bank is the production recognizer. The bank is a reviewed
        # colmag_dtw_template_bank.v1 JSON built by the B2c seed tooling;
        # runtime query points are never y-flipped (the y-flip lives in the
        # bank's seed import). A missing/unloadable bank publishes uncertain
        # payloads instead of falling back to another backend.
        self.dtw_template_bank_path = str(rospy.get_param("~dtw_template_bank_path", "")).strip()
        self.dtw_template_allowed_labels = str(rospy.get_param("~dtw_template_allowed_labels", "1,2,3"))
        self.dtw_template_top_k = int(rospy.get_param("~dtw_template_top_k", 3))
        self.dtw_template_max_distance = float(rospy.get_param("~dtw_template_max_distance", 0.12))
        self.dtw_template_min_margin = float(rospy.get_param("~dtw_template_min_margin", 0.01))
        self.dtw_template_min_confidence = float(rospy.get_param("~dtw_template_min_confidence", 0.30))
        self.dtw_template_bank_state = None

        requested_backend = str(rospy.get_param("~recognizer_backend", "dtw_template_bank")).strip().lower()
        self.backend = self._resolve_backend(requested_backend)
        if self.backend == "dtw_template_bank":
            state = self._dtw_template_bank_state()
            rospy.loginfo("dtw_template_bank status=%s templates=%d labels=%s path=%s",
                          state.get("status"), state.get("template_count", 0),
                          ",".join(state.get("labels", [])), state.get("path", ""))

        self.candidates_pub = rospy.Publisher(self.candidates_topic, String, queue_size=10, latch=True)
        self.label_pub = rospy.Publisher(self.recognized_label_topic, String, queue_size=10, latch=True)
        self.sub = rospy.Subscriber(self.input_topic, String, self._handle_capture, queue_size=10)
        rospy.loginfo("trajectory_symbol_top3_recognizer_node started")
        rospy.loginfo("backend=%s labels=%s", self.backend, ",".join(self.labels))

    def _resolve_backend(self, requested_backend):
        if requested_backend in ("dtw_template_bank", "template_bank"):
            self.rospy.loginfo("dtw_template_bank enabled as backend; bank_path=%s",
                               self.dtw_template_bank_path)
            return "dtw_template_bank"
        self.rospy.logwarn(
            "Unsupported recognizer_backend=%s; using dtw_template_bank only",
            requested_backend,
        )
        return "dtw_template_bank"

    def _import_dtw_bank_tools(self):
        try:
            from colmag_ros import dtw_template_bank_tools as tools
            return tools
        except Exception as exc:
            self.rospy.logwarn("dtw_template_bank_tools import failed: %s", exc)
            return None

    def _dtw_template_bank_state(self):
        """Load the template bank once; a failed load stays unavailable without crashing."""
        if self.dtw_template_bank_state is None:
            tools = self._import_dtw_bank_tools()
            if tools is None:
                self.dtw_template_bank_state = {
                    "status": "unavailable",
                    "reason": "dtw_template_bank_unavailable",
                    "detail": "dtw_template_bank_tools import failed",
                    "path": self.dtw_template_bank_path,
                }
            else:
                self.dtw_template_bank_state = tools.load_template_bank_state(
                    self.dtw_template_bank_path,
                    tools.parse_labels_csv(self.dtw_template_allowed_labels),
                )
        return self.dtw_template_bank_state

    def _recognize_with_dtw_template_bank(self, points):
        """Return (candidates, gate, metadata) from the opt-in template bank.

        Runtime query points are never y-flipped. There is no silent fallback:
        a non-ready bank yields an uncertain gate with empty candidates.
        """
        tools = self._import_dtw_bank_tools()
        state = self._dtw_template_bank_state()
        if tools is None:
            gate = {
                "accepted": False,
                "uncertain": True,
                "uncertainty_reason": "dtw_template_bank_unavailable",
                "top1_confidence": 0.0,
                "top2_confidence": 0.0,
                "best_distance": None,
                "margin": None,
                "max_distance": self.dtw_template_max_distance,
                "min_margin": self.dtw_template_min_margin,
                "min_confidence": self.dtw_template_min_confidence,
            }
            return [], gate, {"dtw_template_bank_status": str(state.get("status", "unavailable"))}
        return tools.recognize_with_template_bank(
            points,
            state,
            top_k=self.dtw_template_top_k,
            max_distance=self.dtw_template_max_distance,
            min_margin=self.dtw_template_min_margin,
            min_confidence=self.dtw_template_min_confidence,
        )

    def _publish_payload(self, payload):
        data = json.dumps(payload, sort_keys=True)
        msg = self.String(data=data)
        self.candidates_pub.publish(msg)
        self.label_pub.publish(msg)
        self.rospy.loginfo("symbol_candidates=%s", data)

    def _publish_invalid_payload(self, reason):
        gate = apply_uncertainty_gate(
            [],
            min_confidence=getattr(self, "min_confidence", 0.70),
            min_margin=getattr(self, "min_margin", 0.20),
            invalid_payload=True,
        )
        payload = {
            "timestamp": time.time(),
            "label": "",
            "confidence": 0.0,
            "reason": "invalid_payload",
            "backend": self.backend,
            "ocr_result_source": "not_used",
            "feature_mode": "none",
            "source_topic": "/colmag/symbol_capture",
            "sample_id": "",
            "sequence_id": 0,
            "candidates": [],
            "invalid_payload_reason": str(reason or "invalid_payload"),
        }
        payload.update(gate)
        self._publish_payload(payload)

    def _parse_capture_message(self, message):
        capture = json.loads(message.data)
        if not isinstance(capture, dict):
            raise TypeError("payload_not_object")
        return capture

    def _recognize_capture(self, capture):
        strokes = normalize_payload_strokes(capture)
        points = flatten_strokes(strokes)
        metadata = stroke_metadata(strokes)
        candidates, dtw_bank_gate, bank_metadata = self._recognize_with_dtw_template_bank(points)
        metadata.update(bank_metadata)
        self.rospy.loginfo(
            "dtw_template_bank status=%s n_candidates=%d",
            metadata.get("dtw_template_bank_status"),
            len(candidates),
        )
        return candidates, dtw_bank_gate, "dtw_template_bank", metadata

    def _build_recognition_payload(
        self,
        capture,
        candidates,
        dtw_bank_gate,
        used_backend,
        metadata,
    ):
        from colmag_ros.symbol_semantics import enrich_candidate_with_semantics
        for c in candidates:
            enrich_candidate_with_semantics(c)

        gate = dtw_bank_gate
        accepted = bool(gate["accepted"])
        published_candidates = list(candidates) if accepted else []
        rejected_candidates = [] if accepted else list(candidates)
        best = candidates[0] if candidates else {"label": "", "confidence": 0.0}
        confidence = float(best.get("confidence", 0.0))
        reason = "threshold_met" if accepted else gate["uncertainty_reason"]
        payload = {
            "timestamp": time.time(),
            "label": best.get("label", ""),
            "confidence": confidence,
            "accepted": accepted,
            "reason": reason,
            "backend": used_backend,
            "ocr_result_source": ocr_result_source_for_backend(used_backend, candidates),
            "feature_mode": feature_mode_for_backend(used_backend),
            "source_topic": "/colmag/symbol_capture",
            "sample_id": capture.get("sample_id", ""),
            "sequence_id": capture.get("sequence_id", 0),
            "candidates": published_candidates,
        }
        if metadata:
            payload.update(metadata)
        payload.update(gate)
        if rejected_candidates:
            payload["rejected_candidates"] = rejected_candidates

        return payload

    def _handle_capture(self, message):
        try:
            capture = self._parse_capture_message(message)
        except ValueError as exc:
            self.rospy.logwarn("failed to parse symbol_capture JSON: %r", exc)
            self._publish_invalid_payload(exc)
            return
        except TypeError:
            self.rospy.logwarn("symbol_capture payload is not a JSON object")
            self._publish_invalid_payload("payload_not_object")
            return

        recognition = self._recognize_capture(capture)
        payload = self._build_recognition_payload(capture, *recognition)
        self._publish_payload(payload)


def main():
    import rospy
    rospy.init_node("trajectory_symbol_top3_recognizer_node")
    TrajectorySymbolTop3RecognizerNode()
    rospy.spin()

if __name__ == "__main__":
    main()

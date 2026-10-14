"""Trajectory sampling and drawing-sample lifecycle for the trajectory dashboard."""

import json
import math
import time

from colmag_ros.dashboard_geometry import derive_orientation_clutch_mode
from colmag_ros.dashboard_points import clean_stroke_points
from colmag_ros.dashboard_sample_lifecycle import (
    build_real_board_sample_record,
    build_dashboard_drawing_zone_capture_payload,
    cleanup_drawing_sample_points,
    real_board_sample_recording_dir,
    safe_sample_recording_name,
    sample_cleanup_metadata,
    trajectory_input_inactive_reason,
    update_drawing_zone_sample_state,
)
from colmag_ros.dashboard_trajectory_processing import stroke_gate_pass
from colmag_ros.param_utils import optional_float
from colmag_ros.trajectory_segment_recorder import (
    STATE_ARMED,
    STATE_CLEARED,
    STATE_IDLE,
    STATE_RECORDING,
    STATE_RECORDING_DONE,
)


def _short_exc(exc, limit=160):
    """One-line, length-bounded description of an exception for UI/log display."""
    text = "%s: %s" % (type(exc).__name__, exc)
    text = " ".join(text.split())  # collapse newlines/whitespace
    return text[:limit]


class DashboardSampleCaptureMixin:
    """Collects trajectory samples, freezes drawing samples, and records real-board samples."""

    def _reset_drawing_sample_state(self):
        self.drawing_sample_state = {
            "active_points": [],
            "frozen_points": [],
            "was_inside": False,
            "phase": "IDLE",
        }
        self.active_drawing_buffer = []
        self.frozen_drawing_sample = []
        self.drawing_zone_trail.clear()

    def _clear_published_sample_state(self):
        self.published_sample_points = []
        self.published_sample_id = ""
        self.published_sample_point_count = 0
        self.published_sample_raw_point_count = 0

    def _clear_sample_record_reference(self):
        self.last_sample_record_path = None
        self.last_sample_record_data = None

    def _write_real_board_sample_record(self, record):
        if not self.config.real_board_sample_recording_enabled:
            return None
        try:
            directory = real_board_sample_recording_dir(self.config.real_board_sample_recording_dir)
            directory.mkdir(parents=True, exist_ok=True)
            sample_id = safe_sample_recording_name(record.get("sample_id", "unknown"))
            output_path = directory / ("%s.json" % sample_id)
            output_path.write_text(json.dumps(record, sort_keys=True, indent=2) + "\n")
            self.last_sample_record_path = output_path
            self.last_sample_record_data = record
            return output_path
        except Exception as exc:
            self.rospy.logwarn("real-board sample recording failed: %s", _short_exc(exc))
            return None

    def _record_real_board_sample(self, capture_payload, raw_points, cleaned_points, metadata):
        record = build_real_board_sample_record(
            capture_payload=capture_payload,
            raw_points=raw_points,
            cleaned_points=cleaned_points,
            cleanup_metadata=metadata,
            candidate_payload=self.current_candidate_payload,
        )
        return self._write_real_board_sample_record(record)

    def _update_real_board_sample_record_candidate(self, candidate_payload):
        if not self.config.real_board_sample_recording_enabled:
            return
        record = self.last_sample_record_data
        path = self.last_sample_record_path
        if not record or path is None:
            return
        sample_id = record.get("sample_id")
        candidate_sample_id = candidate_payload.get("sample_id") if isinstance(candidate_payload, dict) else None
        if sample_id and candidate_sample_id and str(sample_id) != str(candidate_sample_id):
            return
        record["latest_candidate_payload"] = candidate_payload
        record["latest_candidate_may_be_stale"] = False
        self._write_real_board_sample_record(record)

    def _sync_drawing_sample_buffers(self):
        self.active_drawing_buffer = list(self.drawing_sample_state.get("active_points") or [])
        self.frozen_drawing_sample = list(self.drawing_sample_state.get("frozen_points") or [])

    def _sample_lifecycle_phase(self):
        if self.captured_sample_point_count:
            return "RECOGNIZED"
        phase = str((self.drawing_sample_state or {}).get("phase") or "IDLE")
        if phase == "PEN_UP_PENDING" and not self.active_drawing_buffer:
            return "IDLE"
        return phase

    def _freeze_current_drawing_sample(self):
        if self.frozen_drawing_sample:
            return
        if not self.active_drawing_buffer:
            return
        self.drawing_sample_state = update_drawing_zone_sample_state(
            self.drawing_sample_state,
            None,
            inside_drawing_zone=False,
            input_active=True,
            inside_control_zone=True,
            append_point=False,
            force_freeze=True,
        )
        self._sync_drawing_sample_buffers()

    def _start_new_drawing_after_recognized_if_needed(self, inside_drawing_zone, writing):
        if not (inside_drawing_zone and writing):
            return
        if not self.captured_sample_point_count:
            return
        if not self.frozen_drawing_sample:
            return
        self._reset_drawing_sample_state()
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self._clear_published_sample_state()
        self._clear_sample_record_reference()
        self.current_candidate_payload = None
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.trajectory_candidates = []
        self.preview_interaction.new_drawing()

    def _ingest_trajectory_sample(self, data, pt):
        """Pure trajectory viewer: update the trail + live status only.

        Never publishes any intent, task command, or confirmed label. Points that
        are invalid, or that the preview stroke gate marks as non-writing, update
        the live status and current marker but are not appended to the drawn
        trail; instead they insert a stroke break so the next writing point
        starts a fresh segment. Existing history is never cleared here.
        """
        decision = self._classify_trajectory_sample(data, pt)
        inside_drawing_zone, inside_control_zone = (
            self._update_trajectory_sample_buffers(pt, decision)
        )
        self._update_trajectory_sample_status(
            data,
            pt,
            decision,
            inside_drawing_zone,
            inside_control_zone,
        )

        if self.config.trajectory_recording_enabled:
            self._feed_recorder(
                pt,
                decision["z"],
                decision["timestamp"],
                decision["valid"],
                decision["writing"],
            )

        if (
            self.config.candidate_preview_enabled
            and self.config.auto_recognize
            and decision["writing"]
            and inside_drawing_zone
        ):
            self.root.after(0, self._schedule_auto_recognize)

    def _classify_trajectory_sample(self, data, pt):
        valid = bool(data.get("valid", True))
        z_val = optional_float(data.get("z"))
        input_inactive_reason = trajectory_input_inactive_reason(
            valid,
            z_val,
            z_min=self.config.input_z_min,
            z_max=self.config.input_z_max,
            gate_enabled=self.config.input_valid_gate_enabled,
        )
        raw_input_active = input_inactive_reason == ""
        if raw_input_active:
            self._input_active_streak += 1
        else:
            self._input_active_streak = 0
        input_active = raw_input_active and self._input_active_streak >= self.config.input_reactivate_samples

        # Instantaneous XY speed from the previous sample (for speed gating).
        ts = data.get("timestamp", data.get("stamp"))
        speed = None
        if self._prev_sample is not None:
            px, py, pts_t = self._prev_sample
            if isinstance(ts, (int, float)) and isinstance(pts_t, (int, float)):
                dt = ts - pts_t
                if dt > 0:
                    speed = math.hypot(pt[0] - px, pt[1] - py) / dt
        self._prev_sample = (pt[0], pt[1], ts if isinstance(ts, (int, float)) else None)

        writing_gate = stroke_gate_pass(
            self.config.stroke_gate_mode, z_val, speed,
            z_min=self.config.gate_z_min, z_max=self.config.gate_z_max,
            min_speed=self.config.gate_min_speed, max_speed=self.config.gate_max_speed,
        )
        writing = input_active and writing_gate
        return {
            "valid": valid,
            "z": z_val,
            "timestamp": ts,
            "input_active": input_active,
            "input_inactive_reason": input_inactive_reason,
            "writing": writing,
        }

    def _update_trajectory_sample_buffers(self, pt, decision):
        valid = decision["valid"]
        input_active = decision["input_active"]
        input_inactive_reason = decision["input_inactive_reason"]
        writing = decision["writing"]
        self.latest_point = pt
        self.latest_valid = valid
        self.latest_input_active = input_active
        self.latest_input_inactive_reason = "" if input_active else (input_inactive_reason or "reactivating")
        self.latest_writing = writing
        self.trajectory_sample_count += 1

        inside_drawing_zone = False
        inside_control_zone = False
        if input_active:
            inside_drawing_zone = self._trajectory_point_in_drawing_zone(pt)
            inside_control_zone = self._trajectory_point_in_control_zone(pt)
            self._start_new_drawing_after_recognized_if_needed(inside_drawing_zone, writing)
            prev_frozen = bool(self.drawing_sample_state.get("frozen_points"))
            prev_was_inside = bool(self.drawing_sample_state.get("was_inside"))
            self.drawing_sample_state = update_drawing_zone_sample_state(
                self.drawing_sample_state,
                pt,
                inside_drawing_zone,
                input_active=True,
                inside_control_zone=inside_control_zone,
                append_point=writing,
            )
            self._sync_drawing_sample_buffers()
            if writing and inside_drawing_zone and not prev_frozen:
                self.drawing_zone_trail.append(pt)
            elif prev_was_inside and not inside_drawing_zone and not prev_frozen:
                if self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                    self.drawing_zone_trail.append(None)
        else:
            prev_was_inside = bool(self.drawing_sample_state.get("was_inside"))
            self.drawing_sample_state = update_drawing_zone_sample_state(
                self.drawing_sample_state,
                pt,
                inside_drawing_zone=False,
                input_active=False,
                append_point=False,
            )
            self._sync_drawing_sample_buffers()
            if prev_was_inside and self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                self.drawing_zone_trail.append(None)
        self.latest_inside_drawing_zone = bool(inside_drawing_zone)
        self.latest_inside_control_zone = bool(inside_control_zone)

        if writing:
            self.trail.append(pt)
        elif self.config.gate_invalid_breaks_stroke or not input_active:
            # Break the drawn stroke (don't append the point, don't clear history).
            if self.trail and self.trail[-1] is not None:
                self.trail.append(None)
            if self.drawing_zone_trail and self.drawing_zone_trail[-1] is not None:
                self.drawing_zone_trail.append(None)
        return inside_drawing_zone, inside_control_zone

    def _update_trajectory_sample_status(
        self,
        data,
        pt,
        decision,
        inside_drawing_zone,
        inside_control_zone,
    ):
        valid = decision["valid"]
        z_val = decision["z"]
        input_active = decision["input_active"]
        writing = decision["writing"]
        z_txt = "%.4f" % z_val if z_val is not None else "-"
        xyz = "x/y/z: %.4f / %.4f / %s" % (pt[0], pt[1], z_txt)
        idx = data.get("sample_index")
        self.latest_sample_index = idx
        count_txt = "Samples: %d" % self.trajectory_sample_count
        if idx is not None:
            count_txt += " (idx %s)" % idx
        valid_txt = "Valid: %s | input active: %s | writing: %s" % (
            "true" if valid else "false",
            "yes" if input_active else "no",
            "yes" if writing else "no")
        filt = data.get("filter_mode", "-")
        input_txt = "Input: active %s | drawing %s | control %s | reason %s" % (
            "yes" if input_active else "no",
            "yes" if inside_drawing_zone else "no",
            "yes" if inside_control_zone else "no",
            self.latest_input_inactive_reason or "-",
        )

        self.root.after(0, lambda t=valid_txt: self.traj_valid_var.set(t))
        self.root.after(0, lambda t=xyz: self.traj_xyz_var.set(t))
        self.root.after(0, lambda t=count_txt: self.traj_count_var.set(t))
        self.root.after(0, lambda t=input_txt: self.traj_input_gate_var.set(t))
        self.root.after(0, lambda t=filt: self.traj_filter_var.set("Filter: %s" % t))
        self._update_orientation_clutch_mode()
        self._refresh_operator_status()
        self._schedule_ui_refresh()

    def _update_orientation_clutch_mode(self, hovered_button=None):
        if hovered_button is None:
            hovered_button = ""
            if (
                self.latest_input_active
                and self.latest_inside_control_zone
                and self.config.is_trajectory_mode
                and self.config.candidate_preview_enabled
            ):
                hovered_button = self._board_cursor_preview_button()
        mode = derive_orientation_clutch_mode(
            self.latest_input_active,
            self.latest_inside_drawing_zone,
            hovered_button,
        )
        self.root.after(
            0,
            lambda t=mode: self.traj_clutch_mode_var.set("Mode: %s" % t),
        )
        return mode

    # --- Preview-only segment recording: display only, no publish ---
    def _start_recording(self):
        """Start button: clear the previous segment and record a new one.

        Preview-only. Clears the drawn trail + candidates + preview confirm and
        arms the segment recorder. Recording begins on the first writing sample
        and stops after ``record_duration_sec`` (or z-lift when enabled). Never
        publishes anything.
        """
        if not self.config.trajectory_recording_enabled:
            return
        # Reuse the clear path so trail/candidates/confirm reset consistently.
        self._clear_trajectory_trace()
        self.recorded_segment = []
        self.segment_recorder.start()
        self.preview_interaction.recording()
        self._update_candidate_ui()
        self._schedule_ui_refresh()

    def _feed_recorder(self, pt, z_val, ts, valid, writing):
        """Feed one live sample into the segment recorder (recording mode only)."""
        rec = self.segment_recorder
        if rec.state not in (STATE_ARMED, STATE_RECORDING):
            return
        now = ts if isinstance(ts, (int, float)) else time.time()
        state = rec.feed(pt[0], pt[1], now, z=z_val, valid=valid, writing=writing)
        if state == STATE_RECORDING_DONE:
            self._on_recording_done()
        elif state == STATE_RECORDING:
            self.preview_interaction.recording()
            self._update_candidate_ui()

    def _on_recording_done(self):
        """A complete segment is captured. Optionally auto-recognize it."""
        self.recorded_segment = self.segment_recorder.segment()
        self.preview_interaction.recording_done()
        self._update_candidate_ui()
        self._schedule_ui_refresh()
        if self.config.candidate_preview_enabled and self.config.auto_recognize:
            self._recognize_current_stroke()

    def _active_recognition_points(self):
        """Points the recognizer should use.

        Real-board canvas controls keep live cursor movement and button hover in
        ``trail`` for operator feedback, but recognition uses the central
        drawing-zone buffer only.

        When segment recording is enabled and configured to recognize only the
        completed segment, use the recorded segment; otherwise use the live trail.
        Finalizes an in-progress recording first so Recognize always works on a
        complete segment.
        """
        if self.config.is_trajectory_mode and self._use_canvas_reachable_preview_controls():
            frozen = self.frozen_drawing_sample
            if frozen:
                return list(frozen)
            return list(self.active_drawing_buffer)
        rec = self.segment_recorder
        if (self.config.trajectory_recording_enabled
                and self.config.recognize_only_completed_segment
                and rec is not None):
            if rec.is_recording:
                rec.finish_now(reason="recognize")
                self.recorded_segment = rec.segment()
            return list(self.recorded_segment or [])
        return self.trail

    def _recording_status_text(self):
        rec = self.segment_recorder
        if not self.config.trajectory_recording_enabled or rec is None:
            return "Recording: disabled"
        state_map = {
            STATE_IDLE: "idle (press S/Start)",
            STATE_ARMED: "armed — waiting for pen down",
            STATE_RECORDING: "recording…",
            STATE_RECORDING_DONE: "segment ready (%d pts, stop=%s)" % (
                len(rec.points), rec.stop_reason or "-"),
            STATE_CLEARED: "cleared",
        }
        return "Recording: %s" % state_map.get(rec.state, rec.state)

    def _set_captured_sample_display(self, sample_id, points, raw_point_count=None):
        points = list(points or [])
        self.captured_path = points
        self.captured_sample_id = str(sample_id or "unknown")
        self.captured_sample_point_count = len(points)
        self.captured_sample_at = time.time()
        self.published_sample_points = list(points)
        self.published_sample_id = self.captured_sample_id
        self.published_sample_point_count = len(points)
        if raw_point_count is None:
            raw_point_count = len(points)
        self.published_sample_raw_point_count = int(raw_point_count)
        text = "Published sample: %s | raw %d pts -> published %d pts" % (
            self.captured_sample_id,
            int(self.published_sample_raw_point_count),
            self.captured_sample_point_count)
        self.root.after(0, lambda t=text: self.traj_captured_sample_var.set(t))
        self.root.after(0, lambda t=text: self.capture_var.set(t))

    def _publish_trajectory_symbol_capture_from_drawing_zone(self):
        self._freeze_current_drawing_sample()
        raw_points = clean_stroke_points(self._active_recognition_points())
        points = cleanup_drawing_sample_points(
            raw_points,
            min_point_delta=self.config.board_sample_min_point_delta,
            max_points=self.config.board_sample_max_points,
            enabled=self.config.board_sample_cleanup_enabled,
        )
        if len(points) < self.config.candidate_min_points:
            self.preview_interaction.too_few_points()
            text = "Too few published drawing points: %d / %d (raw %d)" % (
                len(points), self.config.candidate_min_points, len(raw_points))
            self.root.after(0, lambda t=text: self.traj_captured_sample_var.set(t))
            self.root.after(0, lambda t=text: self.capture_var.set(t))
            self._update_candidate_ui()
            return False
        now = time.time()
        sample_id = "dashboard_traj_%d" % int(now * 1000)
        sequence_id = int(self.trajectory_sample_count)
        cleanup_meta = sample_cleanup_metadata(
            raw_points,
            points,
            min_point_delta=self.config.board_sample_min_point_delta,
            max_points=self.config.board_sample_max_points,
            enabled=self.config.board_sample_cleanup_enabled,
        )
        payload = build_dashboard_drawing_zone_capture_payload(
            points,
            sample_id,
            sequence_id,
            now,
            self.config.trajectory_topic,
            drawing_zone=self._drawing_zone_canvas_rect(),
            raw_points=raw_points,
            cleanup_metadata=cleanup_meta,
        )
        payload["drawing_zone_sample_frozen"] = bool(self.frozen_drawing_sample)
        self.last_capture_sample_id = sample_id
        self.last_capture_sequence_id = sequence_id
        self._set_captured_sample_display(sample_id, points, raw_point_count=len(raw_points))
        self._record_real_board_sample(payload, raw_points, points, cleanup_meta)
        self.symbol_capture_pub.publish(self.String(data=json.dumps(payload, sort_keys=True)))
        self.preview_interaction.capture_published()
        self._update_candidate_ui()
        self._publish_dashboard_state()
        return True

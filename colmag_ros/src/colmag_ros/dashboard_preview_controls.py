"""Preview, confirm, and candidate display controls for the trajectory dashboard."""

import json
import time

from colmag_ros.dashboard_candidate_display import (
    PreviewCandidateDisplayState,
    derive_operator_workflow_stage,
    format_candidate_debug_details,
    format_candidate_result_summary,
    format_candidate_rows_for_display,
    format_operator_action_status,
    format_operator_recognition_view,
    format_operator_workflow,
    preview_candidate_ui_texts,
    summarize_candidates,
)
from colmag_ros.dashboard_controller_input import (
    build_confirmed_label_payload,
    candidate_payload_key,
    preview_button_availability,
    should_suppress_repeated_confirm,
)
from colmag_ros.dashboard_dwell_status import format_interaction_instruction
from colmag_ros.dashboard_points import build_ocr_stroke_points, clean_stroke_points
from colmag_ros.dashboard_sample_lifecycle import cleanup_drawing_sample_points
from colmag_ros.semantic_library import preview_line


class DashboardPreviewControlsMixin:
    """Handles preview buttons, pointer dwell, confirm/reject, and the candidate panel."""

    def _on_clear_button(self):
        if self.config.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("X")
        else:
            self._clear_trajectory_trace()
            self.root.after(0, lambda: self.ui_state_var.set("State: WAITING (Cleared)"))

    def _on_recognize_button(self):
        self.root.after(0, lambda: self.ui_state_var.set("State: Recognition is automatic; wait for stable trajectory."))

    def _on_confirm_publish_button(self):
        self._confirm_rank_by_dwell(1)

    def _confirm_rank_by_dwell(self, selected_rank):
        if self.confirm_pub is None or not self.config.integrated_confirm_enabled:
            return
        payload = self.current_candidate_payload
        if should_suppress_repeated_confirm(self.last_confirm_key, payload, selected_rank):
            return

        confirmed, reason = build_confirmed_label_payload(payload, selected_rank)
        if confirmed is None:
            self.rospy.logwarn("dashboard dwell confirm skipped: %s", reason)
            return

        self.last_confirm_key = "%s:rank_%d" % (candidate_payload_key(payload), int(selected_rank))
        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = int(selected_rank)
        self.last_command_intent = "CONFIRM_RANK_%d" % int(selected_rank)
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Confirmed: '%s' by dashboard dwell" % self.last_confirmed_label))
        self._publish_dashboard_state()
        self.rospy.loginfo("dashboard dwell confirmed_label=%s", msg.data)

    def _on_reject_button(self):
        if self.config.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("A")
        else:
            self._clear_candidates("Rejected by REJECT button")
            self.root.after(0, lambda: self.ui_state_var.set("State: BLOCKED (Rejected)"))

    def _on_resample_button(self):
        if self.config.demo_input_mode == "ocr_canvas":
            self._handle_ocr_action("B")
        else:
            self._clear_trajectory_trace()
            self.root.after(0, lambda: self.ui_state_var.set("State: Ready for new sample."))

    def _semantic_preview_text(self):
        """Preview-only meaning of the top candidate (display only, no dispatch)."""
        if not self.config.semantic_preview_enabled:
            return "Meaning: -"
        cands = self.trajectory_candidates
        if not cands:
            return "Meaning: -"
        return "Meaning: %s" % preview_line(cands[0])

    # --- Preview-only OCR/candidate bridge: display only, no publish ---
    def _cancel_candidate_recognize_job(self):
        if self._candidate_recognize_job is None:
            return
        try:
            self.root.after_cancel(self._candidate_recognize_job)
        except self.tk.TclError:
            pass
        self._candidate_recognize_job = None

    def _schedule_auto_recognize(self):
        if not (self.config.candidate_preview_enabled and self.config.auto_recognize):
            return
        self._cancel_candidate_recognize_job()
        idle_ms = max(50, int(self.config.ocr_debounce_sec * 1000))
        self._candidate_recognize_job = self.root.after(idle_ms, self._recognize_current_stroke)

    def _clear_trajectory_trace(self):
        self._cancel_candidate_recognize_job()
        self.trail.clear()
        self._reset_drawing_sample_state()
        self.latest_point = None
        self._prev_sample = None
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
        self.preview_interaction.clear()
        if self.segment_recorder is not None:
            self.segment_recorder.clear()
            self.recorded_segment = []
        self._update_candidate_ui()
        self.root.after(0, lambda: self.traj_captured_sample_var.set("Captured sample: none"))
        self.root.after(0, lambda: self.capture_var.set("Cleared trajectory and captured sample"))
        self._schedule_ui_refresh()

    def _reject_preview_candidate(self):
        self._cancel_candidate_recognize_job()
        self.trajectory_candidates = []
        self.current_candidate_payload = None
        self.captured_path = []
        self.captured_sample_id = ""
        self.captured_sample_point_count = 0
        self.captured_sample_at = None
        self._clear_published_sample_state()
        self._clear_sample_record_reference()
        self.last_capture_sample_id = None
        self.last_capture_sequence_id = None
        self.preview_interaction.reject()
        self._update_candidate_ui()
        self.root.after(0, lambda: self.traj_captured_sample_var.set("Captured sample: none"))
        self.root.after(0, lambda: self.capture_var.set("Rejected candidate and captured sample"))
        self._schedule_ui_refresh()

    def _preview_confirm_candidate(self):
        label = self._top_preview_candidate_label()
        if not self.preview_interaction.confirm(label, bool(self.trajectory_candidates)):
            self._update_candidate_ui()
            return
        # The trajectory-preview C confirm also publishes the confirmed
        # label (board red cursor / mouse preview), not just the preview UI. The
        # publish itself is gated inside the helper (double gate + de-dup).
        self._publish_preview_confirmed_label()
        self._update_candidate_ui()
        self._schedule_ui_refresh()

    def _external_confirm_active(self):
        """True when the trajectory-mode dwell confirm should confirm the external
        ROS /colmag/symbol_candidates payload (e.g. dtw_template_bank) instead of
        the dashboard's in-process preview candidates. Falls back to internal
        (returns False) when the source is not selected or no external candidates
        are available."""
        return (
            self.config.is_trajectory_mode
            and self.config.confirm_candidate_source == "external_symbol_candidates"
            and bool((self.current_candidate_payload or {}).get("candidates"))
        )

    def _publish_preview_confirmed_label(self):
        if self._external_confirm_active():
            result = self.preview_confirm_publisher.publish_external_payload(
                confirm_pub=self.confirm_pub,
                integrated_confirm_enabled=self.config.integrated_confirm_enabled,
                candidate_payload=self.current_candidate_payload,
                last_confirm_key=self.last_confirm_key,
                expected_sample_id=self.last_capture_sample_id,
                expected_sequence_id=self.last_capture_sequence_id,
            )
        else:
            result = self.preview_confirm_publisher.publish_rank_one(
                confirm_pub=self.confirm_pub,
                integrated_confirm_enabled=self.config.integrated_confirm_enabled,
                trajectory_candidates=self.trajectory_candidates,
                last_confirm_key=self.last_confirm_key,
            )
        if not result.published:
            return
        self.last_confirm_key = result.confirm_key
        self.last_confirmed_label = result.label
        self.last_selected_rank = result.selected_rank
        self.last_command_intent = result.command_intent
        self.root.after(0, lambda: self.confirm_var.set(
            "Confirmed preview '%s' -> /colmag/confirmed_label" % self.last_confirmed_label))
        self._publish_dashboard_state()

    def _top_preview_candidate_label(self):
        if self._external_confirm_active():
            tops = summarize_candidates(self.current_candidate_payload)
            if tops:
                return str(tops[0][1] or "")
        if not self.trajectory_candidates:
            return ""
        return str(self.trajectory_candidates[0].get("label", "") or "")

    def _drawing_sample_ready(self):
        raw_points = clean_stroke_points(self._active_recognition_points())
        points = cleanup_drawing_sample_points(
            raw_points,
            min_point_delta=self.config.board_sample_min_point_delta,
            max_points=self.config.board_sample_max_points,
            enabled=self.config.board_sample_cleanup_enabled,
        )
        return len(points) >= self.config.candidate_min_points

    def _preview_button_states(self):
        has_sample = self._drawing_sample_ready()
        has_candidate = bool(self._top_preview_candidate_label())
        has_clearable_content = bool(
            clean_stroke_points(self.trail)
            or clean_stroke_points(self.active_drawing_buffer)
            or clean_stroke_points(self.frozen_drawing_sample)
            or self.captured_path
            or self.current_candidate_payload
            or self.trajectory_candidates
            or self.recorded_segment
        )
        return preview_button_availability(
            has_sample, has_candidate, has_clearable_content,
        )

    def _handle_preview_button_action(self, button_id):
        button_id = str(button_id or "").upper()
        if not self._preview_button_enabled(button_id):
            self._update_ocr_toolbar_visuals()
            return False
        if button_id == "X":
            self._clear_trajectory_trace()
        elif button_id == "A":
            self._reject_preview_candidate()
        elif button_id == "B":
            if self._external_confirm_active() or self.config.confirm_candidate_source == "external_symbol_candidates":
                self._publish_trajectory_symbol_capture_from_drawing_zone()
            else:
                self._recognize_current_stroke()
        elif button_id == "C":
            self._preview_confirm_candidate()
        self._update_ocr_toolbar_visuals()
        return True

    def _preview_button_enabled(self, button_id):
        button_id = str(button_id or "").upper()
        return self._preview_button_states().get(button_id, False)

    def _set_operator_recognition_view(
        self, payload, status, sample_ready, result_var, mapping_var, row_vars,
    ):
        view = format_operator_recognition_view(
            payload, status=status, sample_ready=sample_ready)
        self.root.after(0, lambda t=view["headline"]: result_var.set(t))
        self.root.after(0, lambda t=view["mapping"]: mapping_var.set(t))
        for index, text in enumerate(view["rows"]):
            self.root.after(0, lambda i=index, t=text: row_vars[i].set(t))

    def _refresh_operator_status(self):
        sample_ready = self._drawing_sample_ready()
        top_candidate = self._top_preview_candidate_label()
        interaction_state = self.preview_interaction.state
        text = format_operator_action_status(
            interaction_state=interaction_state,
            confirmed_label=self.preview_interaction.confirmed_label,
            top_candidate_label=top_candidate,
            sample_ready=sample_ready,
        )
        self.root.after(0, lambda t=text: self.traj_operator_status_var.set(t))
        stage = derive_operator_workflow_stage(
            interaction_state=interaction_state,
            has_sample=sample_ready,
            has_candidate=bool(top_candidate),
        )
        workflow = format_operator_workflow(stage, self.config.interaction_profile)
        self.root.after(0, lambda t=workflow: self.traj_workflow_var.set(t))
        self._set_operator_recognition_view(
            self.current_candidate_payload,
            interaction_state,
            sample_ready,
            self.traj_operator_result_var,
            self.traj_operator_mapping_var,
            self.traj_operator_candidate_vars,
        )

    def _update_candidate_ui(self):
        status = self.preview_interaction.candidate_status
        live_points = len(build_ocr_stroke_points(self.trail))
        raw_points = len(clean_stroke_points(self.active_drawing_buffer))
        frozen_points = len(clean_stroke_points(self.frozen_drawing_sample))
        published_points = int(self.published_sample_point_count)
        backend = self.recognition_backend_label
        external_payload = (
            self.current_candidate_payload
            if (
                self.config.is_trajectory_mode
                and self.config.confirm_candidate_source == "external_symbol_candidates"
            )
            else None
        )
        display_state = PreviewCandidateDisplayState(
            status=status,
            point_count=live_points,
            backend=backend,
            raw_point_count=raw_points,
            frozen_point_count=frozen_points,
            published_point_count=published_points,
            sample_lifecycle_phase=self._sample_lifecycle_phase(),
            published_sample_raw_point_count=int(self.published_sample_raw_point_count),
            board_sample_cleanup_enabled=self.config.board_sample_cleanup_enabled,
            result_payload=self.current_candidate_payload,
            external_candidate_payload=external_payload,
            hover_button=self.dwell_timer.button,
            hover_source=self.hover_source,
            dwell_progress=self.dwell_timer.progress,
            hover_progress_enabled=self.config.hover_progress_enabled,
            preview_interaction_state=self.preview_interaction.state,
            preview_confirmed_label=self.preview_interaction.confirmed_label,
            trajectory_candidates=self.trajectory_candidates,
            sample_ready=self._drawing_sample_ready(),
        )
        texts = preview_candidate_ui_texts(display_state)
        recording_txt = self._recording_status_text()
        semantic_txt = self._semantic_preview_text()
        self.root.after(0, lambda t=texts["status"]: self.traj_candidate_status_var.set(t))
        self.root.after(0, lambda t=recording_txt: self.traj_recording_var.set(t))
        self.root.after(0, lambda t=semantic_txt: self.traj_semantic_var.set(t))
        self.root.after(0, lambda t=texts["backend"]: self.traj_backend_var.set(t))
        self.root.after(0, lambda t=texts["recognizer_detail"]: self.traj_recognizer_detail_var.set(t))
        self.root.after(0, lambda t=texts["points"]: self.traj_candidate_points_var.set(t))
        self.root.after(0, lambda t=texts["sample"]: self.traj_sample_state_var.set(t))
        self.root.after(0, lambda t=texts["cleanup"]: self.traj_sample_cleanup_var.set(t))
        self.root.after(0, lambda t=texts["result"]: self.traj_result_var.set(t))
        debug_text = format_candidate_debug_details(
            self.current_candidate_payload)
        self.root.after(
            0, lambda t=debug_text: self.traj_candidate_debug_var.set(t))
        self.root.after(
            0,
            lambda t=texts["operator_result"]: self.traj_operator_result_var.set(t),
        )
        self.root.after(0, lambda t=texts["progress"]: self.traj_recognition_progress_var.set(t))
        self.root.after(0, lambda t=texts["interaction"]: self.traj_interaction_var.set(t))
        self.root.after(0, lambda t=texts["confirm"]: self.traj_confirm_var.set(t))
        self.root.after(0, lambda t=texts["operator_status"]: self.traj_operator_status_var.set(t))
        for i, txt in enumerate(texts["candidate_rows"]):
            self.root.after(0, lambda i=i, t=txt: self.traj_candidate_vars[i].set(t))
        self._refresh_operator_status()

    def _sync_external_candidate_display(self, data, backend_text):
        tops = summarize_candidates(data)
        self.trajectory_candidates = [
            {"rank": rank, "label": label, "confidence": confidence}
            for rank, label, confidence in tops
        ]
        display_rows = format_candidate_rows_for_display(data, count=3)
        has_display_candidates = any(not row.endswith("-") for row in display_rows)
        self.preview_interaction.external_candidates(
            has_candidates=bool(tops),
            uncertain=bool(data.get("uncertain") or data.get("accepted") is False),
        )
        self.root.after(0, lambda: self.traj_preview_mode_var.set(
            "Mode: external DTW trajectory recognition"))
        self.root.after(0, lambda t=backend_text: self.traj_backend_var.set(t))
        self.root.after(0, lambda: self.traj_recognizer_detail_var.set(
            "Feature: trajectory_dtw | Source: /colmag/symbol_candidates"))
        status_text = "Candidate status: external candidates ready"
        if self.preview_interaction.candidate_status == "uncertain":
            status_text = "Candidate status: uncertain / redraw"
        elif not tops:
            status_text = "Candidate status: no accepted candidate"
        progress_text = "Progress: external candidates received"
        if self.preview_interaction.candidate_status == "uncertain":
            progress_text = "Progress: rejected by DTW gate"
        elif not has_display_candidates:
            progress_text = "Progress: no candidate"
        self.root.after(0, lambda t=status_text: self.traj_candidate_status_var.set(t))
        self.root.after(0, lambda t=format_candidate_result_summary(data): self.traj_result_var.set(t))
        self.root.after(
            0,
            lambda t=format_candidate_debug_details(data): self.traj_candidate_debug_var.set(t),
        )
        self.root.after(0, lambda t=progress_text: self.traj_recognition_progress_var.set(t))
        for i, text in enumerate(display_rows):
            self.root.after(0, lambda i=i, t=text: self.traj_candidate_vars[i].set(t))
        self._refresh_operator_status()

    def _ocr_confirm_enabled(self):
        return self.ocr_panel_state == "CONFIRM_PENDING" and bool(summarize_candidates(self.current_candidate_payload))

    def _ocr_state_hint_text(self):
        return format_interaction_instruction(self.config.interaction_profile)

    def _refresh_mouse_operator_presentation(self):
        hover_button = str(self.dwell_timer.button or "").upper()
        if hover_button in ("B", "C", "A", "X"):
            mode = "HOVER — %s" % hover_button
        elif self.ocr_panel_state == "DRAWING":
            mode = "DRAW"
        else:
            mode = "IDLE"
        tops = summarize_candidates(self.current_candidate_payload)
        if self.last_confirmed_label:
            stage = "act"
        elif self.ocr_panel_state == "WAITING":
            stage = "start"
        else:
            stage = derive_operator_workflow_stage(
                interaction_state=self.ocr_panel_state,
                has_sample=bool(self._collect_ocr_points()),
                has_candidate=bool(tops),
            )
        if self.ocr_panel_state == "BLOCKED":
            status = "Stopped"
        elif self.ocr_panel_state == "CONFIRM_PENDING" and tops:
            status = "Review the result, then choose C / A / X"
        elif self.ocr_panel_state == "RECOGNIZING":
            status = "Wait for recognition"
        elif self.ocr_panel_state == "DRAWING":
            status = "Draw a character in the center."
        elif self.last_confirmed_label:
            status = "Confirmed: %s" % self.last_confirmed_label
        else:
            status = "Hover B to start drawing"
        self.root.after(0, lambda t=mode: self.mouse_mode_var.set("Mode: %s" % t))
        self.root.after(
            0,
            lambda t=format_operator_workflow(stage, "mouse"): self.mouse_workflow_var.set(t),
        )
        self.root.after(0, lambda t=status: self.mouse_operator_status_var.set(t))
        self._set_operator_recognition_view(
            self.current_candidate_payload,
            self.ocr_panel_state,
            False,
            self.mouse_operator_result_var,
            self.mouse_operator_mapping_var,
            self.mouse_operator_candidate_vars,
        )
        debug_text = format_candidate_debug_details(
            self.current_candidate_payload)
        self.root.after(
            0, lambda t=debug_text: self.mouse_candidate_debug_var.set(t))

    def _preview_pointer_tracking_enabled(self):
        return (
            self.config.is_trajectory_mode
            and self.config.candidate_preview_enabled
            and self.config.hover_buttons_enabled
        )

    def _preview_dwell_active(self):
        """Preview dwell loop should run when either the live board cursor or the
        mouse can drive dwell activation."""
        return (
            self._preview_pointer_tracking_enabled()
            and (self.config.hover_dwell_enabled or self.config.board_cursor_dwell_enabled)
        )

    def _preview_button_at_event(self, event):
        if not self._preview_pointer_tracking_enabled():
            return ""
        try:
            x_root = float(event.x_root)
            y_root = float(event.y_root)
            stage_x = x_root - float(self.ocr_stage_card.winfo_rootx())
            stage_y = y_root - float(self.ocr_stage_card.winfo_rooty())
        except (AttributeError, TypeError, ValueError, self.tk.TclError):
            return ""
        return self._resolve_preview_button_at_stage_xy(stage_x, stage_y)

    def _handle_preview_pointer_motion(self, event):
        button_id = self._preview_button_at_event(event)
        if button_id:
            if button_id != self.dwell_timer.button:
                self._set_ocr_hover_button(button_id)
        elif self.dwell_timer.button:
            self._clear_ocr_hover_button()

    def _handle_preview_pointer_leave(self, event):
        if self._preview_pointer_tracking_enabled():
            self._clear_ocr_hover_button()

    def _handle_preview_pointer_click(self, event):
        if not (self._preview_pointer_tracking_enabled() and self.config.hover_click_enabled):
            return None
        if not self.latest_input_active:
            return None
        button_id = self._preview_button_at_event(event)
        if not button_id or not self._preview_button_enabled(button_id):
            return None
        self._set_ocr_hover_button(button_id)
        return "break" if self._handle_preview_button_action(button_id) else None

    def _process_preview_dwell(self):
        """Preview dwell driven by the live board cursor first, mouse as fallback.

        The red current point is the primary interaction cursor: when it enters a
        button hit zone dwell starts, and after ``control_dwell_sec`` the same safe
        preview handler that a click uses is fired. Board hover is preferred over
        mouse hover so the demo works without a mouse; the choice is deterministic
        (board wins ties). One trigger per dwell entry: the timer re-arms only
        when the effective button changes (i.e. after leaving the zone).
        """
        now = time.time()
        timer = self.dwell_timer
        if not self.latest_input_active:
            self.hover_source = "none"
            timer.reset()
            self.dwell_state = timer.dwell_state(enabled=False)
            self._refresh_dwell_status()
            self._update_ocr_toolbar_visuals()
            self._update_orientation_clutch_mode("")
            self._schedule_ui_refresh()
            self._publish_dashboard_state()
            return
        board_button = self._board_cursor_preview_button() if self.config.board_cursor_dwell_enabled else ""
        mouse_button = self.mouse_hover_button if self.config.hover_dwell_enabled else ""
        # Deterministic: board cursor is primary in the demo, mouse is the fallback.
        if board_button:
            button_id, source = board_button, "board"
        elif mouse_button:
            button_id, source = mouse_button, "mouse"
        else:
            button_id, source = "", "none"
        self.hover_source = source
        # Restart dwell only when the effective button changes, so staying on
        # one button does not re-trigger and leaving/re-entering re-arms it.
        if button_id != timer.button:
            timer.start(button_id, now)
        self._update_orientation_clutch_mode(button_id)

        enabled = bool(button_id) and self._preview_button_enabled(button_id)
        if not enabled:
            timer.hold()
        elif timer.advance(now, self.config.control_dwell_sec):
            self._handle_preview_button_action(button_id)
        self.dwell_state = timer.dwell_state(enabled)
        self._refresh_dwell_status()
        self._update_ocr_toolbar_visuals()
        self._schedule_ui_refresh()
        self._publish_dashboard_state()

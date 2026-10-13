"""Gamepad and rank-confirm input for the trajectory dashboard."""

import json
import time

from colmag_ros.dashboard_candidate_display import summarize_candidates
from colmag_ros.dashboard_controller_input import (
    CONTROLLER_LAYOUT_GAMEPAD,
    GAMEPAD_ARROW_BUTTONS,
    GAMEPAD_MODE_BLOCKED,
    GAMEPAD_MODE_CONFIRM_PENDING,
    GAMEPAD_MODE_DIGIT,
    GAMEPAD_MODE_MOTION,
    build_confirmed_label_payload,
    build_gamepad_confirmed_label_payload,
    candidate_payload_key,
    should_suppress_repeated_confirm,
    update_controller_mode,
    update_dwell_state,
)


class DashboardGamepadMixin:
    """Handles gamepad dwell activation, digit mode, and rank confirmation."""

    def _handle_gamepad_dwell_activation(self, button_id):
        button_id = str(button_id or "").upper()
        now = time.time()
        has_candidates = bool(summarize_candidates(self.current_candidate_payload))

        if button_id == "X":
            self.controller_mode = GAMEPAD_MODE_MOTION
            self.digit_mode_expires_at = None
            self._clear_candidates("Cleared/unblocked by X dwell")
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._publish_dashboard_state()
            return

        if button_id == "B":
            transition = update_controller_mode(self.controller_mode, "B", now, self.digit_mode_sec, has_candidates=has_candidates)
            self.controller_mode = transition["mode"]
            self.digit_mode_expires_at = transition["digit_expires_at"]
            self.root.after(0, lambda: self.confirm_var.set("Digit mode armed by B dwell"))
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._refresh_dwell_status()
            self._publish_dashboard_state()
            return

        if button_id == "C":
            if self.controller_mode != GAMEPAD_MODE_CONFIRM_PENDING:
                return
            self._confirm_rank_by_gamepad_c()
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            return

        if button_id in GAMEPAD_ARROW_BUTTONS:
            if not self.gamepad_publish_motion_buttons:
                return
            self._publish_gamepad_button_label(button_id)
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            return

        if button_id == "A":
            self._publish_gamepad_button_label(button_id)
            self.controller_mode = GAMEPAD_MODE_BLOCKED
            self.digit_mode_expires_at = None
            self.dwell_state = update_dwell_state({}, "", now, self.dwell_confirm_sec)
            self._publish_dashboard_state()

    def _update_digit_mode_timeout(self, now):
        if self.controller_layout != CONTROLLER_LAYOUT_GAMEPAD:
            return
        if self.controller_mode == GAMEPAD_MODE_DIGIT and self.digit_mode_expires_at is not None and now >= self.digit_mode_expires_at:
            self.controller_mode = GAMEPAD_MODE_CONFIRM_PENDING
            self.digit_mode_expires_at = None
            self.root.after(0, lambda: self.confirm_var.set("Digit window ended; C confirms rank 1"))

    def _publish_gamepad_button_label(self, button_id):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        confirmed, reason = build_gamepad_confirmed_label_payload(button_id, self.controller_mode)
        if confirmed is None:
            self.rospy.logwarn("gamepad dwell confirm skipped: %s", reason)
            return

        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = None
        self.last_command_intent = str(confirmed.get("command_intent", ""))
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Gamepad: %s -> %s" % (button_id, self.last_command_intent)))
        self._publish_dashboard_state()
        self.rospy.loginfo("gamepad dwell confirmed_label=%s", msg.data)

    def _confirm_rank_by_gamepad_c(self):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
            return
        payload = self.current_candidate_payload
        if should_suppress_repeated_confirm(self.last_confirm_key, payload, 1):
            return

        confirmed, reason = build_gamepad_confirmed_label_payload("C", self.controller_mode, payload)
        if confirmed is None:
            self.rospy.logwarn("gamepad C confirm skipped: %s", reason)
            return

        self.last_confirm_key = "%s:rank_1" % candidate_payload_key(payload)
        self.last_confirmed_label = str(confirmed.get("label", ""))
        self.last_selected_rank = 1
        self.last_command_intent = str(confirmed.get("command_intent", ""))
        msg = self.String(data=json.dumps(confirmed, sort_keys=True))
        self.confirm_pub.publish(msg)
        self.root.after(0, lambda: self.confirm_var.set("Confirmed rank 1 '%s' by C dwell" % self.last_confirmed_label))
        self._publish_dashboard_state()
        self.rospy.loginfo("gamepad C confirmed_label=%s", msg.data)

    def _confirm_rank_by_dwell(self, selected_rank):
        if self.confirm_pub is None or not self.integrated_confirm_enabled:
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

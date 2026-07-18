"""Small helper for dashboard preview-confirm publishing.

This module keeps the trajectory-preview confirmed-label payload/publish logic
outside the oversized dashboard UI module. It has no ROS master dependency:
tests can inject a String-like factory and a publisher-like object.
"""

import json


def build_mouse_toolbar_stop_payload(timestamp):
    """Build the mouse A/STOP confirmation using the valid C4 cancel label."""
    return {
        "timestamp": float(timestamp),
        "label": "X",
        "confidence": 1.0,
        "confirmed": True,
        "confirmed_by": "ocr_canvas_toolbar_dwell",
        "selected_button": "A",
        "command_intent": "STOP_OR_CANCEL",
        "controls_real_robot": False,
        "gazebo_only": True,
        "test_only": False,
        "source_topic": "/colmag/confirmed_label",
    }


def should_publish_mouse_toolbar_stop(panel_state):
    """Suppress repeated A/STOP dispatch while the mouse toolbar is blocked."""
    return str(panel_state or "").strip().upper() != "BLOCKED"


def external_payload_is_fresh(payload, expected_sample_id, expected_sequence_id):
    """Stale guard for external ROS symbol_candidates confirmation.

    When an expected id is provided (from the latest symbol_capture), the
    payload's matching id must equal it; unknown/empty expected ids skip that
    check (no reference yet). This prevents confirming candidates that belong to
    an older stroke than the one currently on the board.
    """
    if not isinstance(payload, dict):
        return False
    for key, expected in (
        ("sample_id", expected_sample_id),
        ("sequence_id", expected_sequence_id),
    ):
        if expected in (None, ""):
            continue
        if str(payload.get(key, "")) != str(expected):
            return False
    return True


class PreviewConfirmPublishResult:
    def __init__(
        self,
        published=False,
        reason="",
        label="",
        confirm_key="",
        selected_rank=None,
        command_intent="",
        message_data="",
    ):
        self.published = bool(published)
        self.reason = str(reason or "")
        self.label = str(label or "")
        self.confirm_key = str(confirm_key or "")
        self.selected_rank = selected_rank
        self.command_intent = str(command_intent or "")
        self.message_data = str(message_data or "")


class PreviewConfirmPublisher:
    """Build and publish the top preview candidate as a confirmed-label payload."""

    def __init__(
        self,
        string_factory,
        build_confirmed_label_payload,
        should_suppress_repeated_confirm,
        candidate_payload_key,
        logger=None,
    ):
        self._string_factory = string_factory
        self._build_confirmed_label_payload = build_confirmed_label_payload
        self._should_suppress_repeated_confirm = should_suppress_repeated_confirm
        self._candidate_payload_key = candidate_payload_key
        self._logger = logger

    def _warn(self, *args):
        if self._logger is not None:
            self._logger.logwarn(*args)

    def _info(self, *args):
        if self._logger is not None:
            self._logger.loginfo(*args)

    def publish_rank_one(
        self,
        confirm_pub,
        integrated_confirm_enabled,
        trajectory_candidates,
        last_confirm_key,
    ):
        if confirm_pub is None or not integrated_confirm_enabled:
            return PreviewConfirmPublishResult(reason="disabled")
        if not trajectory_candidates:
            self._warn("preview confirm skipped: no candidate")
            return PreviewConfirmPublishResult(reason="no_candidate")

        payload = {"candidates": list(trajectory_candidates)}
        if self._should_suppress_repeated_confirm(last_confirm_key, payload, 1):
            return PreviewConfirmPublishResult(reason="suppressed")

        confirmed, reason = self._build_confirmed_label_payload(payload, 1)
        if confirmed is None:
            self._warn("preview confirm skipped: %s", reason)
            return PreviewConfirmPublishResult(reason=reason)

        confirmed["confirmed_by"] = "magnetic_dashboard_preview_dwell"
        label = str(confirmed.get("label", ""))
        confirm_key = "%s:rank_1" % self._candidate_payload_key(payload)
        message_data = json.dumps(confirmed, sort_keys=True)
        confirm_pub.publish(self._string_factory(data=message_data))
        self._info("preview confirmed_label=%s", message_data)
        return PreviewConfirmPublishResult(
            published=True,
            label=label,
            confirm_key=confirm_key,
            selected_rank=1,
            command_intent="CONFIRM_RANK_1",
            message_data=message_data,
        )

    def publish_external_payload(
        self,
        confirm_pub,
        integrated_confirm_enabled,
        candidate_payload,
        last_confirm_key,
        selected_rank=1,
        expected_sample_id=None,
        expected_sequence_id=None,
    ):
        """Confirm from a full external /colmag/symbol_candidates payload.

        Unlike ``publish_rank_one`` (which rewraps an internal candidate list and
        tags ``confirmed_by=magnetic_dashboard_preview_dwell``), this preserves
        the external recognizer payload metadata (``backend``, ``sample_id``,
        ``sequence_id``, ``candidates``) and keeps the
        ``build_confirmed_label_payload`` ``confirmed_by=magnetic_dashboard_dwell``.
        Used by trajectory-mode dashboard dwell confirm when
        ``confirm_candidate_source=external_symbol_candidates`` so the confirmed
        label is authoritatively the ``dtw_template_bank`` recognizer output.
        """
        if confirm_pub is None or not integrated_confirm_enabled:
            return PreviewConfirmPublishResult(reason="disabled")
        if not isinstance(candidate_payload, dict) or not candidate_payload.get("candidates"):
            self._warn("external confirm skipped: no candidate")
            return PreviewConfirmPublishResult(reason="no_candidate")
        if not external_payload_is_fresh(candidate_payload, expected_sample_id, expected_sequence_id):
            self._warn("external confirm skipped: stale candidates")
            return PreviewConfirmPublishResult(reason="stale")
        if self._should_suppress_repeated_confirm(last_confirm_key, candidate_payload, selected_rank):
            return PreviewConfirmPublishResult(reason="suppressed")

        confirmed, reason = self._build_confirmed_label_payload(candidate_payload, selected_rank)
        if confirmed is None:
            self._warn("external confirm skipped: %s", reason)
            return PreviewConfirmPublishResult(reason=reason)

        # Do NOT override confirmed_by: build_confirmed_label_payload already sets
        # magnetic_dashboard_dwell and preserves the recognizer backend/ids.
        label = str(confirmed.get("label", ""))
        confirm_key = "%s:rank_%d" % (
            self._candidate_payload_key(candidate_payload),
            int(selected_rank),
        )
        message_data = json.dumps(confirmed, sort_keys=True)
        confirm_pub.publish(self._string_factory(data=message_data))
        self._info("external confirmed_label=%s", message_data)
        return PreviewConfirmPublishResult(
            published=True,
            label=label,
            confirm_key=confirm_key,
            selected_rank=int(selected_rank),
            command_intent="CONFIRM_RANK_%d" % int(selected_rank),
            message_data=message_data,
        )

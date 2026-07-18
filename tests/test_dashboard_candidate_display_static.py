"""Static/unit tests for ROS-free dashboard candidate display helpers.

No ROS master, Tk GUI, Gazebo, serial, real board, controller manager, or real
FR3 runtime is started.
"""

import subprocess
import sys
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "colmag_ros" / "scripts"
DASHBOARD = REPO / "colmag_ros" / "scripts" / "magnetic_trajectory_dashboard_node.py"
HELPER = REPO / "colmag_ros" / "scripts" / "dashboard_candidate_display.py"

if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import dashboard_candidate_display as display  # noqa: E402


class CandidateDisplayHelperTests(unittest.TestCase):
    def test_rank_rows_preserve_c2c3_display_only_semantics(self):
        payload = {
            "backend": "dtw_template_bank",
            "candidates": [
                {"rank": 1, "label": "1", "confidence": 0.91},
                {"rank": 2, "label": "O", "confidence": 0.82},
                {"rank": 3, "label": "C", "confidence": 0.73},
            ],
        }
        self.assertEqual(display.format_rank_rows(payload), [
            "Rank 1: 1 | MOVE_LEFT | 0.910",
            "Rank 2: O | ORBIT_SMALL / CIRCLE_PREVIEW | 0.820",
            "Rank 3: C | COMPLIANT_DEMO_SAFE / SOFT_DESCEND_PREVIEW | 0.730",
        ])

    def test_rank_rows_support_exact_8_symbol_set(self):
        expected = {
            "1": "MOVE_LEFT",
            "2": "HEXAGON_TRAJECTORY",
            "3": "MOVE_RIGHT",
            "V": "HOVER_APPROACH",
            "O": "ORBIT_SMALL / CIRCLE_PREVIEW",
            "X": "STOP_OR_CANCEL",
            "A": "HOME_OR_READY",
            "C": "COMPLIANT_DEMO_SAFE / SOFT_DESCEND_PREVIEW",
        }
        for label, semantic in expected.items():
            with self.subTest(label=label):
                rows = display.format_rank_rows({
                    "candidates": [{"rank": 1, "label": label, "confidence": 0.5}],
                })
                self.assertEqual(rows[0], "Rank 1: %s | %s | 0.500" % (label, semantic))

    def test_missing_confidence_task_backend_and_candidate_fallbacks(self):
        payload = {
            "candidates": [
                {"rank": 1, "label": "Z"},
                {"rank": 2, "label": "Q", "confidence": "bad"},
                {"rank": "bad", "label": "X", "confidence": 0.9},
            ],
        }
        self.assertEqual(display.candidate_backend_status_text({}), "Backend: unknown")
        self.assertEqual(display.summarize_candidates(payload), [(1, "Z", 0.0)])
        self.assertEqual(display.format_rank_rows(payload), [
            "Rank 1: Z | 0.000",
            "Rank 2: -",
            "Rank 3: -",
        ])

    def test_backend_display_for_dtw_template_bank(self):
        self.assertEqual(display.backend_display_name("dtw_template_bank"), "DTW template bank")
        self.assertEqual(display.backend_display_name("dtw"), "DTW")
        self.assertEqual(display.backend_display_name("unexpected"), "Unknown")
        text = display.candidate_backend_status_text({
            "backend": "dtw_template_bank",
            "feature_mode": "trajectory_dtw",
        })
        self.assertEqual(
            text,
            "Backend: dtw_template_bank | Feature: trajectory_dtw",
        )

    def test_external_dtw_result_summary_shows_gate_details(self):
        payload = {
            "backend": "dtw_template_bank",
            "feature_mode": "trajectory_dtw",
            "accepted": True,
            "reason": "threshold_met",
            "best_distance": 0.04321,
            "margin": 0.01876,
            "candidates": [
                {"rank": 1, "label": "2", "confidence": 0.82},
                {"rank": 2, "label": "1", "confidence": 0.44},
            ],
        }
        self.assertEqual(
            display.format_candidate_result_summary(payload),
            "Result: accepted | top-1 2 0.820 | reason threshold_met | "
            "best_distance 0.043 | margin 0.019",
        )

    def test_uncertain_dtw_display_uses_rejected_top_k(self):
        payload = {
            "backend": "dtw_template_bank",
            "feature_mode": "trajectory_dtw",
            "accepted": False,
            "uncertain": True,
            "uncertainty_reason": "dtw_margin_too_small",
            "best_distance": 0.052,
            "margin": 0.004,
            "candidates": [],
            "rejected_candidates": [
                {"rank": 1, "label": "3", "confidence": 0.70},
                {"rank": 2, "label": "1", "confidence": 0.68},
            ],
        }
        self.assertEqual(
            display.format_candidate_result_summary(payload),
            "Result: uncertain | top-1 3 0.700 | reason dtw_margin_too_small | "
            "best_distance 0.052 | margin 0.004",
        )
        self.assertEqual(display.format_candidate_rows_for_display(payload, count=3), [
            "Rank 1 rejected: 3 | MOVE_RIGHT | 0.700",
            "Rank 2 rejected: 1 | MOVE_LEFT | 0.680",
            "Rank 3: -",
        ])

    def test_recognizer_status_and_label_formatting_stay_compact(self):
        texts = display.recognizer_status_texts({
            "backend": "dtw_template_bank",
            "status": "ready",
            "recognition_labels": ["1", "2", "3", "V", "O", "X", "A", "C"],
        })
        self.assertEqual(texts["mode"], "Mode: DTW template bank")
        self.assertEqual(texts["model"], "Model: n/a")
        self.assertEqual(texts["status"], "Status: ready")
        self.assertEqual(texts["labels"], "Labels: 1, 2, 3, V, O, X, A, C")

    def test_preview_candidate_ui_texts_preserve_legacy_strings(self):
        texts = display.preview_candidate_ui_texts(
            display.PreviewCandidateDisplayState(
                status="ready",
                point_count=7,
                backend="DTW",
                hover_button="C",
                hover_source="board",
                dwell_progress=0.42,
                hover_progress_enabled=True,
                preview_interaction_state="ready",
                preview_confirmed_label="2",
                trajectory_candidates=[
                    {"rank": 1, "label": "2", "confidence": 0.912},
                    {"rank": 2, "label": "3", "confidence": 0.451},
                ],
            )
        )
        self.assertEqual(texts["status"], "Candidate status: candidates ready")
        self.assertEqual(texts["points"], "Points collected: 7")
        self.assertEqual(texts["backend"], "Backend: DTW")
        self.assertEqual(texts["recognizer_detail"], "Feature: trajectory_dtw | Source: /colmag/symbol_candidates")
        self.assertEqual(texts["progress"], "Progress: top candidates ready")
        self.assertEqual(
            texts["interaction"],
            "Interaction: ready | Source: board | Active: C | Dwell: 42%",
        )
        self.assertEqual(texts["confirm"], "Preview confirm: 2")
        self.assertEqual(texts["candidate_rows"], [
            "Candidate 1: 2  score=0.91",
            "Candidate 2: 3  score=0.45",
            "Candidate 3: -",
        ])

    def test_preview_candidate_ui_texts_preserve_fallback_statuses(self):
        texts = display.preview_candidate_ui_texts(
            display.PreviewCandidateDisplayState(
                status="unavailable",
                point_count=0,
                backend="DTW unavailable",
                hover_button="",
                hover_source="",
                dwell_progress=0.75,
                hover_progress_enabled=False,
                preview_interaction_state="collecting",
                preview_confirmed_label="",
                trajectory_candidates=[],
            )
        )
        self.assertEqual(texts["status"], "Candidate status: recognizer unavailable")
        self.assertEqual(texts["backend"], "Backend: DTW unavailable")
        self.assertEqual(texts["recognizer_detail"], "Feature: trajectory_dtw | Source: /colmag/symbol_candidates")
        self.assertEqual(texts["progress"], "Progress: unavailable")
        self.assertEqual(
            texts["interaction"],
            "Interaction: collecting | Source: none | Active: - | Dwell: 0%",
        )
        self.assertEqual(texts["confirm"], "Confirm: disabled in preview")
        self.assertEqual(texts["candidate_rows"], [
            "Candidate 1: -",
            "Candidate 2: -",
            "Candidate 3: -",
        ])

    def test_preview_candidate_ui_texts_always_returns_result_fallback(self):
        texts = display.preview_candidate_ui_texts(
            display.PreviewCandidateDisplayState(
                status="idle",
                point_count=0,
                backend="DTW template bank",
                sample_lifecycle_phase="idle",
                result_payload=None,
            )
        )

        self.assertEqual(texts["result"], "Result: waiting for candidates")
        self.assertEqual(set(texts), {
            "status", "points", "sample", "cleanup", "backend", "recognizer_detail",
            "progress", "interaction", "confirm", "candidate_rows", "result",
            "operator_result", "operator_candidate_rows", "operator_status",
        })
        self.assertIsNone(texts["sample"])
        self.assertIsNone(texts["cleanup"])

    def test_operator_status_formats_existing_action_and_candidate_state(self):
        self.assertEqual(
            display.format_operator_action_status(sample_ready=True),
            "Hover B Recognize",
        )
        self.assertEqual(
            display.format_operator_action_status(
                interaction_state="external_candidates_ready",
                top_candidate_label="2",
            ),
            "Review the result, then choose C / A / X",
        )
        self.assertEqual(
            display.format_operator_action_status(
                interaction_state="preview_confirmed",
                confirmed_label="2",
            ),
            "Confirmed: 2",
        )
        self.assertEqual(
            display.format_operator_action_status(interaction_state="rejected"),
            "Candidate rejected",
        )
        self.assertEqual(
            display.format_operator_action_status(interaction_state="cleared"),
            "Drawing cleared",
        )
        self.assertEqual(
            display.format_operator_action_status(
                interaction_state="symbol_capture_published"
            ),
            "Wait for recognition",
        )

    def test_operator_recognition_summary_hides_backend_reason_details(self):
        text = display.format_operator_recognition_summary({
            "backend": "dtw_template_bank",
            "accepted": True,
            "reason": "threshold_met",
            "best_distance": 0.04321,
            "candidates": [
                {"rank": 1, "label": "2", "confidence": 0.82},
            ],
        })
        self.assertEqual(
            text,
            "Recognized symbol: 2 | Confidence: 82% | Accepted",
        )
        self.assertNotIn("backend", text.lower())
        self.assertNotIn("reason", text.lower())
        self.assertNotIn("distance", text.lower())

    def test_operator_recognition_separates_symbol_mapping_and_debug_details(self):
        payload = {
            "accepted": True,
            "best_distance": 0.04321,
            "candidates": [
                {
                    "rank": 1, "label": "2", "confidence": 0.763,
                    "task": "HEXAGON_TRAJECTORY",
                    "template_bank_name": "bank_a", "template_id": "two_1",
                },
                {"rank": 2, "label": "V", "confidence": 0.11},
                {"rank": 3, "label": "N", "confidence": 0.055},
            ],
        }
        view = display.format_operator_recognition_view(payload)
        self.assertEqual(view["headline"], "Recognized symbol: 2")
        self.assertEqual(view["mapping"], "Mapped action: Hexagon trajectory")
        self.assertEqual(view["rows"], [
            "Confidence: 76% · Accepted",
            "Rank 2: V · confidence 11%",
            "Rank 3: N · confidence 6%",
        ])
        operator_text = " ".join([view["headline"], view["mapping"]] + view["rows"])
        self.assertNotIn("distance", operator_text.lower())
        self.assertNotIn("template", operator_text.lower())
        debug_text = display.format_candidate_debug_details(payload)
        self.assertIn("DTW distance: 0.043", debug_text)
        self.assertIn("Template bank: bank_a", debug_text)
        self.assertIn("Template: two_1", debug_text)
        self.assertIn("Action ID: HEXAGON_TRAJECTORY", debug_text)

    def test_mapped_action_humanizes_ids_without_fabricating_values(self):
        self.assertEqual(
            display.format_mapped_action("HEXAGON_TRAJECTORY"),
            "Hexagon trajectory",
        )
        self.assertEqual(display.format_mapped_action("FR3_READY"), "FR3 ready")
        self.assertEqual(display.format_mapped_action(""), "")
        view = display.format_operator_recognition_view({
            "accepted": True,
            "candidates": [{"rank": 1, "label": "2", "confidence": 0.7}],
        })
        self.assertEqual(view["mapping"], "")

    def test_zero_confidence_alternatives_are_compact_but_nonzero_stays_visible(self):
        view = display.format_operator_recognition_view({
            "accepted": True,
            "candidates": [
                {"rank": 1, "label": "2", "confidence": 0.76},
                {"rank": 2, "label": "V", "confidence": 0.12},
                {"rank": 3, "label": "C", "confidence": 0.0},
            ],
        })
        self.assertEqual(view["rows"], [
            "Confidence: 76% · Accepted",
            "Rank 2: V · confidence 12%",
            "Other candidates: C",
        ])

    def test_recognition_and_status_have_distinct_operator_jobs(self):
        empty = display.format_operator_recognition_view(None)
        ready = display.format_operator_recognition_view(None, sample_ready=True)
        self.assertEqual(empty["headline"], "No candidates yet")
        self.assertEqual(ready["headline"], "Trajectory ready for recognition")
        self.assertEqual(
            display.format_operator_action_status(),
            "Draw a character in the center",
        )
        self.assertEqual(
            display.format_operator_action_status(sample_ready=True),
            "Hover B Recognize",
        )
        self.assertNotEqual(empty["headline"], display.format_operator_action_status())
        self.assertNotEqual(
            ready["headline"], display.format_operator_action_status(sample_ready=True))

    def test_profile_workflows_and_empty_guidance_are_source_aware(self):
        self.assertEqual(
            display.format_operator_workflow("recognize", "real_board"),
            "✓\u00a0Draw  →  ●\u00a0Recognize\nReview  →  Act",
        )
        self.assertEqual(
            display.format_operator_workflow("start", "mouse"),
            "●\u00a0Start\u00a0drawing  →  Draw\nAuto-recognize  →  Review  →  Act",
        )
        self.assertEqual(
            display.format_operator_workflow("review", "mouse"),
            "✓\u00a0Start\u00a0drawing  →  ✓\u00a0Draw\n"
            "✓\u00a0Auto-recognize  →  ●\u00a0Review  →  Act",
        )
        for profile in ("real_board", "mouse"):
            text = display.format_operator_workflow("review", profile)
            self.assertNotIn("●\n", text)
            self.assertNotIn("\n●\n", text)
            self.assertNotIn("● ", text)
        self.assertEqual(
            display.format_operator_candidate_rows(
                ["Candidate 1: -", "Candidate 2: -", "Candidate 3: -"],
                sample_ready=False,
            ),
            ["Draw a character in the center.", "", ""],
        )
        self.assertEqual(
            display.format_operator_candidate_rows([], sample_ready=True),
            ["Hover B Recognize to submit the trajectory.", "", ""],
        )

    def test_preview_candidate_display_state_is_frozen(self):
        state = display.PreviewCandidateDisplayState("idle", 0, "DTW")
        with self.assertRaises(FrozenInstanceError):
            state.status = "ready"

    def test_full_display_state_preserves_sample_and_external_candidate_contract(self):
        payload = {
            "accepted": True,
            "reason": "threshold_met",
            "candidates": [
                {"rank": 1, "label": "1", "confidence": 0.9},
                {"rank": 2, "label": "2", "confidence": 0.8},
                {"rank": 3, "label": "3", "confidence": 0.7},
            ],
        }
        texts = display.preview_candidate_ui_texts(display.PreviewCandidateDisplayState(
            status="ready",
            point_count=5,
            backend="DTW template bank",
            raw_point_count=8,
            frozen_point_count=7,
            published_point_count=6,
            sample_lifecycle_phase="SAMPLE_FROZEN",
            published_sample_raw_point_count=7,
            board_sample_cleanup_enabled=True,
            result_payload=payload,
            external_candidate_payload=payload,
        ))
        self.assertEqual(
            texts["points"],
            "Live points: 5 | Raw drawing: 8 | Frozen raw: 7 | Published sample: 6",
        )
        self.assertEqual(
            texts["sample"],
            "Sample: SAMPLE_FROZEN | Live 5 | Raw 8 | Frozen raw 7 | Published 6",
        )
        self.assertEqual(
            texts["cleanup"],
            "Published sample: raw 7 -> clean 6 | cleanup on | controls excluded",
        )
        self.assertEqual(texts["candidate_rows"], [
            "Rank 1: 1 | MOVE_LEFT | 0.900",
            "Rank 2: 2 | HEXAGON_TRAJECTORY | 0.800",
            "Rank 3: 3 | MOVE_RIGHT | 0.700",
        ])


class DashboardExtractionStaticTests(unittest.TestCase):
    def test_dashboard_imports_and_uses_helper(self):
        text = DASHBOARD.read_text()
        self.assertIn("from colmag_ros.dashboard_candidate_display import", text)
        self.assertIn("format_candidate_result_summary", text)
        self.assertIn("format_candidate_rows_for_display", text)
        self.assertIn("format_rank_rows(data, count=3)", text)
        self.assertIn("preview_candidate_ui_texts(", text)
        self.assertNotIn("def candidate_display_name(", text)
        self.assertNotIn("def candidate_backend_status_text(", text)
        self.assertNotIn("def recognizer_status_texts(", text)

    def test_helper_has_no_ros_tk_dispatch_or_execution_imports(self):
        text = HELPER.read_text()
        for forbidden in (
            "rospy",
            "tkinter",
            "Publisher(",
            "task_dispatcher_node.py",
            "fr3_gazebo_visible_task_bridge_node.py",
            "gazebo_task_executor.py",
            "/colmag/task_command",
            "FollowJointTrajectory",
        ):
            self.assertNotIn(forbidden, text)

    def test_dashboard_confirmation_behavior_not_modified_by_helper(self):
        text = DASHBOARD.read_text()
        for expected in (
            "def _external_confirm_active(",
            "publish_external_payload(",
            "publish_rank_one(",
            "def _confirm_rank_by_dwell(",
            "def _confirm_rank_by_gamepad_c(",
        ):
            self.assertIn(expected, text)

    def test_no_generated_outputs_are_tracked_or_staged(self):
        tracked = subprocess.run(
            ["git", "ls-files", "outputs", "*.joblib", ".pytest_cache", "__pycache__"],
            cwd=str(REPO), capture_output=True, text=True, check=True,
        ).stdout.strip()
        staged = subprocess.run(
            ["git", "diff", "--cached", "--name-only"],
            cwd=str(REPO), capture_output=True, text=True, check=True,
        ).stdout.strip()
        self.assertEqual(tracked, "")
        for forbidden in ("outputs/", ".joblib", ".pytest_cache", "__pycache__"):
            self.assertNotIn(forbidden, staged)


if __name__ == "__main__":
    unittest.main()

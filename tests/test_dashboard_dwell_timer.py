from colmag_ros.dashboard_controller_input import cancelled_preview_hover_state
from colmag_ros.dashboard_dwell_timer import DwellTimer


def test_dwell_fires_once_per_entry():
    timer = DwellTimer()
    timer.start("B", 10.0)
    assert timer.advance(10.5, 1.0) is False
    assert timer.progress == 0.5
    assert timer.advance(11.0, 1.0) is True
    assert timer.advance(12.0, 1.0) is False  # still on B: no refire
    assert (timer.progress, timer.fired) == (1.0, True)

    timer.start("B", 12.0)  # re-entering re-arms
    assert timer.advance(13.0, 1.0) is True


def test_start_without_button_clears_the_entry_time():
    timer = DwellTimer()
    timer.start("C", 5.0)
    timer.advance(5.4, 1.0)
    timer.start("", 6.0)
    assert (timer.button, timer.started_at, timer.progress, timer.fired) == ("", None, 0.0, False)


def test_hold_hides_progress_but_keeps_the_entry_time():
    timer = DwellTimer()
    timer.start("C", 10.0)
    timer.advance(10.5, 1.0)
    timer.hold()
    assert (timer.started_at, timer.progress) == (10.0, 0.0)
    # Re-enabled later, dwell counts from the original entry.
    assert timer.advance(11.0, 1.0) is True


def test_advance_starts_timing_when_no_entry_time():
    timer = DwellTimer(button="A")
    assert timer.advance(3.0, 2.0) is False
    assert (timer.started_at, timer.progress) == (3.0, 0.0)


def test_dwell_state_reports_only_enabled_buttons():
    timer = DwellTimer()
    timer.start("C", 1.0)
    timer.advance(2.0, 1.0)
    assert timer.dwell_state(enabled=True) == {
        "current_button_id": "C", "started_at": 1.0,
        "dwell_progress": 1.0, "activated_button_id": "C"}
    assert timer.dwell_state(enabled=True, report_activation=False)["activated_button_id"] == ""
    assert timer.dwell_state(enabled=False)["current_button_id"] == ""
    assert timer.dwell_state(enabled=False)["dwell_progress"] == 0.0

    timer.reset()
    assert timer.dwell_state(enabled=False) == cancelled_preview_hover_state()

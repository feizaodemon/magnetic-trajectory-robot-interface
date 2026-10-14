"""Path helpers shared by the offline DTW bank scripts."""

from pathlib import Path

from colmag_ros.dtw_dataset_io import resolve_repo_path, seed_files_by_label


def test_resolve_repo_path_keeps_absolute_and_anchors_relative(tmp_path):
    assert resolve_repo_path(tmp_path / "bank.json", "/repo") == tmp_path / "bank.json"
    assert resolve_repo_path("outputs/bank.json", "/repo") == Path("/repo/outputs/bank.json")


def test_seed_files_by_label_sorts_seeds_and_quotes_label_dirs(tmp_path):
    (tmp_path / "1").mkdir()
    (tmp_path / "1" / "b.json").write_text("{}")
    (tmp_path / "1" / "a.json").write_text("{}")
    (tmp_path / "1" / "notes.txt").write_text("")
    (tmp_path / "%3F").mkdir()
    (tmp_path / "%3F" / "q.json").write_text("{}")

    assert seed_files_by_label(tmp_path, ["1", "?", "V"]) == {
        "1": [tmp_path / "1" / "a.json", tmp_path / "1" / "b.json"],
        "?": [tmp_path / "%3F" / "q.json"],
        "V": [],
    }

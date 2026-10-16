"""Leave-one-out evaluation of a DTW template bank (scripts/eval_dtw_bank_loo.py)."""

import importlib.util
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("eval_dtw_bank_loo", REPO / "scripts" / "eval_dtw_bank_loo.py")
loo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(loo)


def _line(dx, dy, n=8, jitter=0.0):
    return [(k * dx / (n - 1) + jitter, k * dy / (n - 1)) for k in range(n)]


def _bank(templates):
    return {"templates": [{"id": "t%d" % i, "label": label, "points": points}
                          for i, (label, points) in enumerate(templates)]}


def test_query_is_never_matched_against_itself():
    # One template per label: with itself removed, "h" can only match "v".
    bank = _bank([("h", _line(1, 0)), ("v", _line(0, 1))])
    result = loo.leave_one_out(bank, labels=["h", "v"])
    assert result["queries"] == 2
    assert result["top1_accuracy"] == 0.0
    assert result["confusion"] == {"h": {"h": 0, "v": 1}, "v": {"h": 1, "v": 0}}


def test_separable_bank_scores_perfectly_and_formats():
    bank = _bank([("h", _line(1, 0)), ("h", _line(1, 0, jitter=0.01)),
                  ("v", _line(0, 1)), ("v", _line(0, 1, jitter=0.01))])
    result = loo.leave_one_out(bank, labels=["h", "v"])
    assert result["top1_accuracy"] == 1.0
    assert result["top3_accuracy"] == 1.0
    assert result["errors"] == []
    assert result["per_label"]["h"] == {"queries": 2, "top1": 1.0}
    text = loo.format_markdown(result)
    assert "| Top-1 accuracy | 100.0% |" in text


def test_min_top1_gate_sets_exit_code(tmp_path, capsys):
    import json
    bank = _bank([("h", _line(1, 0)), ("v", _line(0, 1))])
    bank.update(schema="colmag_dtw_template_bank.v1", model_type="dtw_template_bank",
                labels=["h", "v"], allowed_labels=["h", "v"], preprocess={})
    path = tmp_path / "bank.json"
    path.write_text(json.dumps(bank))
    assert loo.main(["--bank", str(path)]) == 0
    assert loo.main(["--bank", str(path), "--min-top1", "0.5"]) == 1
    capsys.readouterr()

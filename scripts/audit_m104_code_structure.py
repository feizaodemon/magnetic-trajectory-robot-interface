#!/usr/bin/env python3
"""Read-only M104 code-size and function-length audit helper.

Default mode prints a JSON report to stdout and writes nothing. The optional
--write-json path is restricted to outputs/agent_tmp so generated audit reports
stay ignored and repo-local.
"""

import argparse
import ast
import json
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
DEFAULT_FILES = [
    "colmag_ros/scripts/magnetic_trajectory_dashboard_node.py",
    "colmag_ros/scripts/task_dispatcher_node.py",
    "colmag_gazebo_stub/scripts/fr3_gazebo_visible_task_bridge_node.py",
    "colmag_ros/scripts/trajectory_symbol_top3_recognizer_node.py",
    "colmag_ros/scripts/dashboard_confirm_publisher.py",
    "colmag_ros/scripts/m104c4_execution_semantics.py",
    "colmag_ros/scripts/magnetic_ui_state_node.py",
]
DEFAULT_OUTPUT = REPO / "outputs" / "agent_tmp" / "m104_code_structure_audit.json"


def line_count(path):
    return len(path.read_text(errors="ignore").splitlines())


def python_functions(path):
    tree = ast.parse(path.read_text(), filename=str(path))
    functions = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            end_lineno = getattr(node, "end_lineno", node.lineno)
            functions.append(
                {
                    "name": node.name,
                    "line": node.lineno,
                    "lines": end_lineno - node.lineno + 1,
                }
            )
    return sorted(functions, key=lambda item: (-item["lines"], item["line"]))


def file_report(relative_path):
    path = REPO / relative_path
    report = {
        "file": relative_path,
        "exists": path.is_file(),
        "line_count": 0,
        "long_functions": [],
    }
    if not path.is_file():
        return report
    report["line_count"] = line_count(path)
    if path.suffix == ".py":
        report["long_functions"] = [
            function for function in python_functions(path) if function["lines"] >= 80
        ]
    return report


def audit(files):
    return {
        "scope": "M104-R1b-prep/E0 static code-structure audit",
        "writes_by_default": False,
        "files": [file_report(path) for path in files],
    }


def require_agent_tmp(path):
    resolved = path.resolve()
    allowed_root = (REPO / "outputs" / "agent_tmp").resolve()
    try:
        resolved.relative_to(allowed_root)
    except ValueError as exc:
        raise SystemExit("--write-json must stay under outputs/agent_tmp") from exc
    return resolved


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-json", nargs="?", const=str(DEFAULT_OUTPUT), default="")
    parser.add_argument("files", nargs="*", help="Repo-relative files to audit.")
    args = parser.parse_args()

    report = audit(args.files or DEFAULT_FILES)
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)

    if args.write_json:
        output_path = require_agent_tmp((REPO / args.write_json) if not Path(args.write_json).is_absolute() else Path(args.write_json))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(text + "\n")


if __name__ == "__main__":
    main()

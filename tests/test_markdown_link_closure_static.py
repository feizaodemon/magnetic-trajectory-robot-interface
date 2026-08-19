"""Portfolio Markdown inventory and local-link closure checks."""

from pathlib import Path
import re
from urllib.parse import unquote


REPO = Path(__file__).resolve().parents[1]
LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
EXPECTED_READER_DOCS = {
    "CREDITS.md",
    "NOTICE.md",
    "README.md",
    "colmag_ros/README.md",
    "docker/README.md",
    "docker/README_FR3_HARDWARE.md",
    "docs/ARCHITECTURE.md",
    "docs/DUAL_MODE_TELEOPERATION.md",
    "docs/GLOSSARY.md",
    "docs/HARDWARE_BOUNDARIES.md",
    "docs/INTERACTION_PROFILES.md",
    "docs/RUNNING.md",
    "docs/VALIDATION.md",
    "tests/README.md",
}


def _markdown_files():
    return sorted(
        path
        for path in REPO.rglob("*.md")
        if "outputs" not in path.relative_to(REPO).parts
    )


def _without_fenced_code(text):
    kept = []
    fence = None
    for line in text.splitlines():
        marker = line.lstrip()[:3]
        if marker in {"```", "~~~"}:
            fence = None if fence == marker else marker
            continue
        if fence is None:
            kept.append(line)
    return "\n".join(kept)


def _local_targets(path):
    text = _without_fenced_code(path.read_text(encoding="utf-8"))
    for match in LINK_RE.finditer(text):
        raw = match.group(1).strip().strip("<>")
        if not raw or raw.startswith("#") or "://" in raw or raw.startswith("mailto:"):
            continue
        target = unquote(raw.split("#", 1)[0].split("?", 1)[0])
        if target:
            yield target


def _has_exact_case(path):
    relative = path.relative_to(REPO)
    current = REPO
    for part in relative.parts:
        names = {child.name for child in current.iterdir()}
        if part not in names:
            return False
        current = current / part
    return True


def test_reader_document_inventory_is_compact_and_process_files_are_absent():
    actual = {path.relative_to(REPO).as_posix() for path in _markdown_files()}
    assert actual == EXPECTED_READER_DOCS
    for excluded in (
        "AGENTS.md",
        "CLAUDE.md",
        "CODEX_CONTEXT.md",
        "TASK_LOG.md",
        "repo-docs",
        "ClassesSlides",
    ):
        assert not (REPO / excluded).exists()


def test_all_local_markdown_links_and_images_exist_with_exact_case():
    failures = []
    for source in _markdown_files():
        for raw_target in _local_targets(source):
            target = (source.parent / raw_target).resolve()
            try:
                target.relative_to(REPO)
            except ValueError:
                failures.append(f"{source.relative_to(REPO)} -> outside repository: {raw_target}")
                continue
            if not target.exists():
                failures.append(f"{source.relative_to(REPO)} -> missing: {raw_target}")
            elif not _has_exact_case(target):
                failures.append(f"{source.relative_to(REPO)} -> case mismatch: {raw_target}")
    assert not failures, "\n".join(failures)

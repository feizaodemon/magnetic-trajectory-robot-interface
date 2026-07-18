#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export PYTHONDONTWRITEBYTECODE=1
exec python3 "${REPO_ROOT}/scripts/fr3_lab_operator.py" "$@"

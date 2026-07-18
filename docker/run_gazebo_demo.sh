#!/usr/bin/env bash
set -euo pipefail

PROFILE="${1:-mouse-gazebo}"
if [[ "${PROFILE}" != "mouse-gazebo" && "${PROFILE}" != "real-board-gazebo" ]]; then
  echo "Usage: $0 [mouse-gazebo|real-board-gazebo]"
  exit 2
fi
shift || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
export COLMAG_RUNTIME_LOG_DIR="${COLMAG_RUNTIME_LOG_DIR:-${REPO_ROOT}/outputs/runtime_logs/gazebo_${PROFILE}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${COLMAG_RUNTIME_LOG_DIR}/ros"

echo "Profile: ${PROFILE}"
echo "Logs: ${COLMAG_RUNTIME_LOG_DIR}"

exec docker compose -f "${SCRIPT_DIR}/compose.yaml" \
  --profile "${PROFILE}" run --rm "${PROFILE}" "$@"

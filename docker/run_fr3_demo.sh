#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/compose.fr3.yaml"
ENV_FILE="${SCRIPT_DIR}/fr3-hardware.env"
DEFAULT_IMAGE="magnetic-trajectory-interface:fr3-noetic"
REVISION_LABEL="org.opencontainers.image.revision"
SERVICES=(fr3-hardware-dry-run fr3-hardware-active fr3-hardware-active-board)
usage() {
  cat <<EOF
Usage: $0 {status|dry-run|mouse|board|stop}
  status   Show repository, image, gates, and owned-service status
  dry-run  Start the connected no-motion FR3 profile
  mouse    Start the sequential-goal real-FR3 Mouse profile
  board    Start the sequential-goal real-board FR3 profile
  stop     Stop only the owned FR3 Compose services
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

env_value() {
  local key="$1"
  [[ -f "${ENV_FILE}" ]] || return 0
  awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); sub(/\r$/, ""); print; exit}' "${ENV_FILE}"
}

compose() {
  if [[ -f "${ENV_FILE}" ]]; then
    docker compose --env-file "${ENV_FILE}" -f "${COMPOSE_FILE}" "$@"
  else
    docker compose -f "${COMPOSE_FILE}" "$@"
  fi
}

running_services() {
  compose --profile fr3-hardware-dry-run --profile fr3-hardware-active \
    --profile fr3-hardware-active-board ps --format '{{.Service}} {{.Name}}'
}

image_name() {
  local configured
  configured="$(env_value COLMAG_FR3_DOCKER_IMAGE)"
  echo "${configured:-${DEFAULT_IMAGE}}"
}

image_revision() {
  docker image inspect "$1" --format "{{ index .Config.Labels \"${REVISION_LABEL}\" }}"
}

require_active_gates() {
  [[ "$(env_value COLMAG_HARDWARE_EXECUTION_ENABLED)" == "true" ]] ||
    die "COLMAG_HARDWARE_EXECUTION_ENABLED must equal true in ${ENV_FILE}."
  [[ "$(env_value COLMAG_SEND_GOALS)" == "true" ]] ||
    die "COLMAG_SEND_GOALS must equal true in ${ENV_FILE}."
}

require_board_access() {
  local device group gid
  device="$(env_value COLMAG_BOARD_DEVICE)"
  [[ -n "${device}" ]] || die "COLMAG_BOARD_DEVICE must be set in ${ENV_FILE}."
  [[ -c "${device}" ]] || die "COLMAG_BOARD_DEVICE must resolve to a host character device."
  group="$(getent group dialout)" || die "Host dialout group is unavailable."
  gid="$(awk -F: '{print $3; exit}' <<<"${group}")"
  [[ "${gid}" =~ ^[0-9]+$ ]] || die "Host dialout GID could not be resolved."
  export COLMAG_DIALOUT_GID="${gid}"
}

require_start_ready() {
  local robot_ip head dirty image revision running
  [[ -f "${ENV_FILE}" ]] || die "Missing ${ENV_FILE}; create it from the example first."
  robot_ip="$(env_value FR3_ROBOT_IP)"
  case "${robot_ip}" in
    ""|"__REQUIRED_FR3_ROBOT_IP__"|"<"*">") die "FR3_ROBOT_IP is not configured in ${ENV_FILE}." ;;
  esac
  dirty="$(git -C "${REPO_ROOT}" status --porcelain=v1 --untracked-files=all)"
  [[ -z "${dirty}" ]] || die "Repository must be clean before starting an FR3 profile."
  head="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
  image="$(image_name)"
  revision="$(image_revision "${image}" 2>/dev/null)" || die "FR3 image is missing: ${image}"
  [[ -n "${revision}" && "${revision}" != "uncommitted" ]] ||
    die "FR3 image has no committed source revision."
  [[ "${revision}" == "${head}" ]] ||
    die "FR3 image revision ${revision} does not match repository HEAD ${head}."
  running="$(running_services)" || die "Could not inspect owned FR3 Compose services."
  if [[ -n "${running}" ]]; then
    printf 'Error: owned FR3 service is already running:\n%s\n' "${running}" >&2
    echo "Run $0 stop before starting another profile." >&2
    exit 1
  fi
  echo "Repository revision: ${head}"
  echo "FR3 image: ${image}"
  echo "Image revision: ${revision}"
}

start_profile() {
  local profile="$1" service="$2" warning="$3"
  require_start_ready
  echo "${warning}"
  echo "This wrapper does not qualify Gate B or produce a PASS/FAIL verdict."
  compose --profile "${profile}" up "${service}"
}

show_status() {
  local branch head clean image revision match robot execution send gates quota board running
  branch="$(git -C "${REPO_ROOT}" branch --show-current 2>/dev/null || echo unknown)"
  head="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
  [[ -z "$(git -C "${REPO_ROOT}" status --porcelain=v1 --untracked-files=all 2>/dev/null)" ]] && clean=yes || clean=no
  image="$(image_name)"
  robot="$(env_value FR3_ROBOT_IP)"; [[ -n "${robot}" && "${robot}" != "__REQUIRED_FR3_ROBOT_IP__" ]] && robot=set || robot=unset
  execution="$(env_value COLMAG_HARDWARE_EXECUTION_ENABLED)"; send="$(env_value COLMAG_SEND_GOALS)"
  gates="${execution:-unset}/${send:-unset}"
  quota="$(env_value COLMAG_MAX_MOTION_GOALS_PER_PROCESS)"; quota="${quota:-1}"
  board="$(env_value COLMAG_BOARD_DEVICE)"; [[ -n "${board}" ]] && board=set || board=unset
  revision=missing; match=no; running=unavailable
  if docker info >/dev/null 2>&1; then
    revision="$(image_revision "${image}" 2>/dev/null || echo missing)"
    [[ "${revision}" == "${head}" ]] && match=yes
    running="$(running_services 2>/dev/null || echo unavailable)"; [[ -n "${running}" ]] || running=none
  fi
  printf 'Repository: %s\nBranch: %s\nHEAD: %s\nWorking tree clean: %s\n' "${REPO_ROOT}" "${branch}" "${head}" "${clean}"
  printf 'Expected FR3 image: %s\nImage revision: %s\nRevision match: %s\n' "${image}" "${revision}" "${match}"
  printf 'Environment file: %s\nRobot IP: %s\nActive gates (execution/send): %s\nMotion goal quota (0=unlimited): %s\nBoard device: %s\n' \
    "$([[ -f "${ENV_FILE}" ]] && echo yes || echo no)" "${robot}" "${gates}" "${quota}" "${board}"
  printf 'Running owned FR3 services:\n%s\n' "${running}"
}

stop_services() {
  compose --profile fr3-hardware-dry-run --profile fr3-hardware-active \
    --profile fr3-hardware-active-board stop "${SERVICES[@]}"
  local running
  running="$(running_services)" || die "Could not inspect owned FR3 Compose services after stop."
  [[ -n "${running}" ]] && printf 'Remaining owned FR3 services:\n%s\n' "${running}" || echo "Remaining owned FR3 services: none"
}

command="${1:-help}"
[[ "$#" -le 1 ]] || { usage >&2; exit 2; }
case "${command}" in
  help|-h|--help) usage ;;
  status) show_status ;;
  dry-run) start_profile fr3-hardware-dry-run fr3-hardware-dry-run "Connected dry-run only; manual Gate B probes remain required." ;;
  mouse) require_active_gates; start_profile fr3-hardware-active fr3-hardware-active "Starting the sequential-goal real-FR3 Mouse route." ;;
  board) require_active_gates; require_board_access; start_profile fr3-hardware-active-board fr3-hardware-active-board "Starting the sequential-goal real-board FR3 route." ;;
  stop) stop_services ;;
  *) usage >&2; exit 2 ;;
esac

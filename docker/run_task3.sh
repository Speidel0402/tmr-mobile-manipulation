#!/usr/bin/env bash
# Stage the exact image contents on each ROS host, then run the coordinator.
set -Eeo pipefail

image_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
arm_host="${EBIM_ARM_HOST:-aup@172.16.0.100}"
base_host="${EBIM_BASE_HOST:-tmr-user@172.16.0.50}"
remote_root="${EBIM_REMOTE_ROOT:-/tmp/edl_task3_phase2}"
arm_environment="${EBIM_ARM_ENV:-${remote_root}/docker/load_arm_environment.sh}"
mode="execute"

case "${1:-}" in
  preflight|check|execute)
    mode="$1"
    shift
    ;;
esac

ssh_options=(
  -o BatchMode=yes
  -o ConnectTimeout=5
  -o ServerAliveInterval=2
  -o ServerAliveCountMax=3
  -o StrictHostKeyChecking=accept-new
  -o UserKnownHostsFile=/tmp/edl_task3_known_hosts
)

preflight() {
  local required
  for required in \
    base grasp mission tools docker/load_arm_environment.sh \
    docker/load_base_environment.sh docker/check_remote_runtime.sh \
    docker/ssh_wrapper.sh mission/scripts/run_complete_from_start.sh; do
    [[ -e "${image_root}/${required}" ]] || {
      echo "[preflight] missing image payload: ${required}" >&2
      return 1
    }
  done
  bash -n "${image_root}"/docker/*.sh \
    "${image_root}/mission/scripts/run_complete_from_start.sh"
  python3 -m compileall -q \
    "${image_root}/base/scripts" \
    "${image_root}/grasp/scripts" \
    "${image_root}/mission/scripts" \
    "${image_root}/tools"
  echo '{"status":"success","mode":"preflight","physical_motion_commanded":false}'
}

stage_host() {
  local host="$1"
  ssh "${ssh_options[@]}" "${host}" \
    "rm -rf '${remote_root}' && mkdir -p '${remote_root}'"
  tar -C "${image_root}" -czf - base grasp mission tools docker | \
    ssh "${ssh_options[@]}" "${host}" \
      "tar -xzf - -C '${remote_root}'"
}

check_host() {
  local host="$1"
  local role="$2"
  local environment="$3"
  ssh "${ssh_options[@]}" "${host}" \
    "EBIM_RUNTIME_ROLE='${role}' EBIM_RUNTIME_ENV='${environment}' bash '${remote_root}/docker/check_remote_runtime.sh'"
}

if [[ "${mode}" == "preflight" ]]; then
  preflight
  exit 0
fi

# ROS never runs in this container. Each host loads only its native ROS
# distribution after receiving the exact policy files embedded in the image.
stage_host "${arm_host}"
stage_host "${base_host}"

if [[ "${mode}" == "check" ]]; then
  check_host "${arm_host}" arm "${arm_environment}"
  check_host "${base_host}" base "${remote_root}/docker/load_base_environment.sh"
  echo '{"status":"success","mode":"check","physical_motion_commanded":false}'
  exit 0
fi

exec bash "${image_root}/mission/scripts/run_complete_from_start.sh" \
  --arm-host "${arm_host}" \
  --arm-remote-root "${remote_root}" \
  --arm-env "${arm_environment}" \
  --base-host "${base_host}" \
  --base-root "${remote_root}/base" \
  "$@"

#!/usr/bin/env bash
# Stage the exact image contents on each ROS host, then run the coordinator.
set -Eeo pipefail

image_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
arm_host="${EBIM_ARM_HOST:-aup@172.16.0.100}"
base_host="${EBIM_BASE_HOST:-tmr-user@172.16.0.50}"
remote_root="${EBIM_REMOTE_ROOT:-/tmp/edl_task3_phase2}"
arm_environment="${EBIM_ARM_ENV:-/home/aup/tmr_env.sh}"

ssh_options=(
  -o BatchMode=yes
  -o ConnectTimeout=5
  -o ServerAliveInterval=2
  -o ServerAliveCountMax=3
)

stage_host() {
  local host="$1"
  ssh "${ssh_options[@]}" "${host}" \
    "rm -rf '${remote_root}' && mkdir -p '${remote_root}'"
  tar -C "${image_root}" -czf - base grasp mission tools | \
    ssh "${ssh_options[@]}" "${host}" \
      "tar -xzf - -C '${remote_root}'"
}

# ROS never runs in this container. Each host loads only its native ROS
# distribution after receiving the exact policy files embedded in the image.
stage_host "${arm_host}"
stage_host "${base_host}"

exec bash "${image_root}/mission/scripts/run_complete_from_start.sh" \
  --arm-host "${arm_host}" \
  --arm-remote-root "${remote_root}" \
  --arm-env "${arm_environment}" \
  --base-host "${base_host}" \
  --base-root "${remote_root}/base" \
  "$@"

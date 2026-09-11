#!/usr/bin/env bash
# Read-only remote contract check. This script intentionally sends no robot
# motion, gripper, controller-switch, or lifting-column command.
set -eo pipefail

role="${EBIM_RUNTIME_ROLE:?EBIM_RUNTIME_ROLE is required}"
environment="${EBIM_RUNTIME_ENV:?EBIM_RUNTIME_ENV is required}"
[[ -r "${environment}" ]] || {
  echo "runtime environment loader is unavailable: ${environment}" >&2
  exit 70
}
# shellcheck disable=SC1090
source "${environment}"

require_name() {
  local kind="$1"
  local name="$2"
  local values="$3"
  grep -Fxq "${name}" <<<"${values}" || {
    echo "missing ${kind}: ${name}" >&2
    return 1
  }
}

if [[ "${role}" == "arm" ]]; then
  [[ -w /tmp ]] || {
    echo "remote /tmp is not writable for Task 3 runtime state" >&2
    exit 74
  }
  services="$(ros2 service list --no-daemon 2>/dev/null || ros2 service list)"
  actions="$(ros2 action list)"
  topics="$(ros2 topic list --no-daemon 2>/dev/null || ros2 topic list)"

  require_name service /left/controller_manager/list_controllers "${services}"
  require_name service /left_ik/compute_fk "${services}"
  require_name service /left_ik/compute_ik "${services}"
  require_name service /franka_spine_node/get_position "${services}"
  require_name action /left/action_server/ptp_motion "${actions}"
  require_name action /left/gripper/robotiq_gripper_controller/gripper_cmd "${actions}"
  require_name topic /wrist_camera_left/color/image_raw "${topics}"

  timeout 8 ros2 topic echo --once --qos-reliability best_effort \
    /wrist_camera_left/color/image_raw >/dev/null
elif [[ "${role}" == "base" ]]; then
  [[ -w /tmp ]] || {
    echo "remote /tmp is not writable for Task 3 runtime state and locks" >&2
    exit 74
  }
  command -v flock >/dev/null 2>&1 || {
    echo "base runtime dependency is unavailable: flock" >&2
    exit 75
  }
  command -v screen >/dev/null 2>&1 || {
    echo "base runtime dependency is unavailable: screen" >&2
    exit 75
  }
  [[ -d /run/screen ]] || {
    echo "base screen runtime directory is unavailable: /run/screen" >&2
    exit 76
  }
  topics="$(ros2 topic list --no-daemon 2>/dev/null || ros2 topic list)"
  require_name topic /swerve_drive_controller/odom "${topics}"
  require_name topic /swerve_drive_controller/cmd_vel "${topics}"
  timeout 8 ros2 topic echo --once --qos-reliability best_effort \
    /swerve_drive_controller/odom >/dev/null
else
  echo "unknown runtime role: ${role}" >&2
  exit 71
fi

printf '{"status":"success","role":"%s","ros_distro":"%s","ros_domain_id":"%s","rmw":"%s","physical_motion_commanded":false}\n' \
  "${role}" "${ROS_DISTRO:-unknown}" "${ROS_DOMAIN_ID:-unset}" "${RMW_IMPLEMENTATION:-default}"

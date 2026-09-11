#!/usr/bin/env bash
# Reconstruct the deployed arm ROS environment without a private ~/tmr_env.sh.
# This file is sourced by task commands; do not enable nounset here because ROS
# setup files legitimately inspect optional unset variables.
set -e

read_process_value() {
  local pid="$1"
  local wanted="$2"
  local entry
  while IFS= read -r -d '' entry; do
    if [[ "${entry}" == "${wanted}="* ]]; then
      printf '%s\n' "${entry#*=}"
      return 0
    fi
  done <"/proc/${pid}/environ"
  return 1
}

find_robot_pid() {
  local pattern pid
  for pattern in \
    '[/]controller_manager/ros2_control_node.*left' \
    '[f]ranka.*ros2_control_node' \
    '[f]ranka.*robot_state' \
    '[r]os2_control_node'; do
    while IFS= read -r pid; do
      [[ -r "/proc/${pid}/environ" ]] && {
        printf '%s\n' "${pid}"
        return 0
      }
    done < <(pgrep -f "${pattern}" 2>/dev/null || true)
  done
  return 1
}

robot_pid="$(find_robot_pid || true)"
ros_distro="${EBIM_ARM_ROS_DISTRO:-}"
if [[ -z "${ros_distro}" && -n "${robot_pid}" ]]; then
  ros_distro="$(read_process_value "${robot_pid}" ROS_DISTRO || true)"
fi
ros_distro="${ros_distro:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
[[ -r "${ros_setup}" ]] || {
  echo "arm ROS setup is unavailable: ${ros_setup}" >&2
  return 72 2>/dev/null || exit 72
}

# shellcheck disable=SC1090
source "${ros_setup}"

# Import only ROS discovery/runtime variables from a live robot process. Never
# print or copy its complete environment, which may contain unrelated secrets.
if [[ -n "${robot_pid}" ]]; then
  while IFS= read -r -d '' entry; do
    name="${entry%%=*}"
    case "${name}" in
      AMENT_PREFIX_PATH|CMAKE_PREFIX_PATH|COLCON_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH|PATH|ROS_DISTRO|ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE)
        export "${entry}"
        ;;
    esac
  done <"/proc/${robot_pid}/environ"
fi

# Optional, non-secret overlay list for a host where the running process
# environment is not readable. Entries are colon separated setup.bash files.
if [[ -n "${EBIM_ARM_OVERLAYS:-}" ]]; then
  IFS=':' read -r -a overlays <<<"${EBIM_ARM_OVERLAYS}"
  for overlay in "${overlays[@]}"; do
    [[ -r "${overlay}" ]] || {
      echo "configured arm overlay is unavailable: ${overlay}" >&2
      return 73 2>/dev/null || exit 73
    }
    # shellcheck disable=SC1090
    source "${overlay}"
  done
fi

python3 - <<'PY'
import importlib

required = (
    "rclpy",
    "control_msgs",
    "franka_msgs",
    "franka_spine_msgs",
    "moveit_msgs",
    "sensor_msgs",
)
missing = []
for name in required:
    try:
        importlib.import_module(name)
    except Exception as exc:
        missing.append(f"{name}: {exc}")
if missing:
    raise SystemExit("arm ROS Python interfaces unavailable: " + "; ".join(missing))
PY

export EBIM_RESOLVED_ARM_ENV=1

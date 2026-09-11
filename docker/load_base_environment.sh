#!/usr/bin/env bash
# Load the deployed Humble base graph without requiring a policy checkout in a
# fixed home-directory path.
set -e

source /opt/ros/humble/setup.bash
for overlay in \
  "${HOME}/ros2_ws/install/setup.bash" \
  "${HOME}/tmr_navigation/install/setup.bash" \
  "${HOME}/tmr_navigation/install/tmr_local_navigation/share/tmr_local_navigation/local_setup.bash"; do
  [[ ! -r "${overlay}" ]] || source "${overlay}"
done

export ROS_DOMAIN_ID="${TMR_CYCLE_ROS_DOMAIN_ID:-97}"
export ROS_LOCALHOST_ONLY="${TMR_CYCLE_ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${TMR_CYCLE_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -n "${TMR_CYCLE_CYCLONEDDS_URI:-}" ]]; then
  export CYCLONEDDS_URI="${TMR_CYCLE_CYCLONEDDS_URI}"
elif [[ -r "${HOME}/cyclonedds.xml" ]]; then
  export CYCLONEDDS_URI="file://${HOME}/cyclonedds.xml"
else
  unset CYCLONEDDS_URI || true
fi
export PYTHONUNBUFFERED=1

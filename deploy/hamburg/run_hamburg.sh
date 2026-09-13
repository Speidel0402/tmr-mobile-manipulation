#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-check}"
shift || true

case "${MODE}" in
  check)
    exec python3 "${SCRIPT_DIR}/hamburg_preflight.py" "$@"
    ;;
  grasp-check)
    exec python3 "${SCRIPT_DIR}/hamburg_grasp_check.py" "$@"
    ;;
  grasp-test)
    echo "Hamburg physical grasp test is locked: native Gello motion planning and gripper contact feedback are not yet validated." >&2
    echo "Use grasp-check for a read-only object-specific camera and interface check." >&2
    exit 3
    ;;
  mission)
    echo "Hamburg mission is locked: the Shanghai pickup, transport, and placement scripts create ROS participants during motion and depend on motion/camera interfaces not established on this venue." >&2
    echo "The native-interface check is read-only and cannot unlock physical motion; a single-node Hamburg mission and supervised grasp tests are still required." >&2
    exit 3
    ;;
  *)
    echo "usage: $0 {check|grasp-check|grasp-test|mission} [arguments]" >&2
    exit 64
    ;;
esac

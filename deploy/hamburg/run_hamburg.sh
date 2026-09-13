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
  grasp-check-all)
    output_dir="${1:-/tmp/tmr_hamburg_grasp_checks}"
    mkdir -p -- "${output_dir}"
    failed=0
    for object in cup bowl plate; do
      python3 "${SCRIPT_DIR}/hamburg_grasp_check.py" \
        --object "${object}" --output "${output_dir}/${object}.json" \
        --image-output "${output_dir}/${object}.png" || failed=1
    done
    exit "${failed}"
    ;;
  pickup-reset)
    exec python3 "${SCRIPT_DIR}/hamburg_pickup_reset.py" "$@"
    ;;
  geometry-review)
    exec python3 "${SCRIPT_DIR}/geometry_review.py" "$@"
    ;;
  grasp-test)
    exec python3 "${SCRIPT_DIR}/hamburg_grasp_cycle.py" "$@"
    ;;
  mission)
    exec python3 "${SCRIPT_DIR}/hamburg_mission.py" "$@"
    ;;
  site-config)
    exec python3 "${SCRIPT_DIR}/hamburg_site_config.py" "$@"
    ;;
  *)
    echo "usage: $0 {check|grasp-check|grasp-check-all|pickup-reset|geometry-review|grasp-test|mission|site-config} [arguments]" >&2
    exit 64
    ;;
esac

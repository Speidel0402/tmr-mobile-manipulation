#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-check}"
shift || true

case "${MODE}" in
  check)
    exec python3 "${SCRIPT_DIR}/hamburg_preflight.py" "$@"
    ;;
  mission)
    echo "Hamburg mission is intentionally locked: the submitted Shanghai mission creates ROS nodes during active motion and depends on undocumented MoveIt/PTP/spine interfaces." >&2
    echo "Run '${SCRIPT_DIR}/run_hamburg.sh check' and provide the JSON report before enabling physical motion." >&2
    exit 3
    ;;
  *)
    echo "usage: $0 {check|mission} [arguments]" >&2
    exit 64
    ;;
esac

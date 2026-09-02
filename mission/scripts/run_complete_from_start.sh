#!/usr/bin/env bash
# Canonical competition entry: always run the complete mission from the marked
# start.  Mid-mission recovery remains available through its dedicated scripts
# and cannot be selected accidentally through this entry.
set -Eeo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

for argument in "$@"; do
  case "${argument}" in
    --resume-at-pickup-confirmed|--resume-after-cup-held-confirmed|--resume-object-at-pickup-confirmed)
      echo "resume options are not accepted by the from-start entry" >&2
      exit 2
      ;;
  esac
done

exec python3 "${root}/mission/scripts/run_three_object_delivery.py" \
  --execute --fresh-start-confirmed "$@"

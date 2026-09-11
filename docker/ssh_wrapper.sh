#!/usr/bin/env bash
# Apply the same disposable host-key policy to every coordinator SSH process.
# Authentication still uses evaluator-provided keys or SSH agent credentials.
exec /usr/bin/ssh \
  -o StrictHostKeyChecking=accept-new \
  -o UserKnownHostsFile=/tmp/edl_task3_known_hosts \
  "$@"

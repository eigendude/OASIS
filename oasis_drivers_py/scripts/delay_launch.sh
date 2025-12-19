#!/bin/bash
################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See DOCS/LICENSING.md for more information.
#
################################################################################
#
# Helper script to add a delay when ROS nodes are launched on startup, to make
# sure that the system is ready and all nodes will connect without issues.
#

# Enable strict shell mode
set -o errexit
set -o pipefail
set -o nounset

# Determine the delay duration (in seconds) based on system architecture
ARCHITECTURE="$(uname -m)"

case "${ARCHITECTURE}" in
  x86_64)
    DELAY_DURATION="20"
    ;;
  arm*|aarch64)
    DELAY_DURATION="30"
    ;;
  *)
    echo "Warning: Unknown architecture '${ARCHITECTURE}', defaulting delay to 30 seconds." >&2
    DELAY_DURATION="30"
    ;;
esac

# Get the system uptime
UPTIME_SECONDS="$(awk '{print int($1)}' /proc/uptime)"

# If the system has been up for less than the delay duration, wait the remaining time
if (( UPTIME_SECONDS < DELAY_DURATION )); then
  SLEEP_DURATION=$(( DELAY_DURATION - UPTIME_SECONDS ))
  echo "System uptime is ${UPTIME_SECONDS} seconds. Delaying for ${SLEEP_DURATION} seconds..."
  sleep "${SLEEP_DURATION}"
else
  echo "System uptime is ${UPTIME_SECONDS} seconds. No delay needed."
fi

NETWORK_CHECK_INTERVAL="5"

network_ready() {
  local gateway

  if [[ -z "$(ip -o addr show scope global up | head -n 1)" ]]; then
    return 1
  fi

  gateway="$(ip route show default 2>/dev/null | awk '/default/ {print $3; exit}')"
  if [[ -z "${gateway}" ]]; then
    return 1
  fi

  ping -c 1 -W 1 "${gateway}" >/dev/null 2>&1
}

echo "Checking for network readiness (required for Zenoh)..."
until network_ready; do
  echo "Network not ready yet. Waiting ${NETWORK_CHECK_INTERVAL} seconds..."
  sleep "${NETWORK_CHECK_INTERVAL}"
done
echo "Network ready."

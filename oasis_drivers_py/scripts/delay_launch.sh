#!/bin/bash
################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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
  local default_route
  local gateway
  local route_target

  if ! ip -o addr show scope global up 2>/dev/null | grep -q .; then
    echo "No global UP address found." >&2
    return 1
  fi

  default_route="$(ip route show default 2>/dev/null | head -n 1)"
  if [[ -z "${default_route}" ]]; then
    echo "No default route found." >&2
    return 1
  fi

  gateway="$(
    awk '{
      for (i = 1; i <= NF; i++) {
        if ($i == "via") {
          print $(i + 1)
          exit
        }
      }
    }' <<< "${default_route}"
  )"

  if [[ -n "${gateway}" ]]; then
    if ip route get "${gateway}" >/dev/null 2>&1; then
      return 0
    fi
  fi

  route_target="1.1.1.1"

  if ! ip route get "${route_target}" >/dev/null 2>&1; then
    if [[ -n "${gateway}" ]]; then
      echo "Route lookup failed for ${gateway} and ${route_target}." >&2
    else
      echo "Route lookup failed for ${route_target}." >&2
    fi
    return 1
  fi
}

if [[ "${OASIS_SKIP_NETWORK_READY_CHECK:-0}" == "1" ]]; then
  echo "Skipping network readiness check."
else
  echo "Checking for network readiness (required for Zenoh)..."
  until network_ready; do
    echo "Network not ready yet. Waiting ${NETWORK_CHECK_INTERVAL} seconds..."
    sleep "${NETWORK_CHECK_INTERVAL}"
  done
  echo "Network ready."
fi

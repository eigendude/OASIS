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

# Determine the delay duration (in seconds)
# TODO: This should depend on the system being used
DELAY_DURATION="30"

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

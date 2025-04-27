#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

# Directory where systemd services are installed
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system/"

# Directory for udev rules
UDEV_RULE_DIRECTORY="/etc/udev/rules.d"

#
# Uninstall systemd services
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for OASIS_PACKAGE in \
    oasis_control \
    oasis_drivers_py \
    oasis_perception_py \
    oasis_visualization \
  ; do
    for SYSTEMD_SERVICE in "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd/"*.service; do
      SYSTEMD_TIMER="${SYSTEMD_SERVICE%.service}.timer"

      # Get service filename
      SERVICE_FILE="$(basename -- "${SYSTEMD_SERVICE}")"

      # Get timer filename
      TIMER_FILE="$(basename -- "${SYSTEMD_TIMER}")"

      # Skip files that weren't installed
      if [ ! -f "${SYSTEMD_SERVICE_DIRECTORY}/${SERVICE_FILE}" ]; then
        echo "Not installed: ${SERVICE_FILE}"
        continue
      fi

      echo "Uninstalling ${SERVICE_FILE}"

      # Disable systemd service/timer
      sudo rm -f "${SYSTEMD_SERVICE_DIRECTORY}/multi-user.target.wants/${SERVICE_FILE}"
      sudo rm -f "${SYSTEMD_SERVICE_DIRECTORY}/timers.target.wants/${TIMER_FILE}"

      # Remove systemd service/timer
      sudo rm -f "${SYSTEMD_SERVICE_DIRECTORY}/${SERVICE_FILE}"
      sudo rm -f "${SYSTEMD_SERVICE_DIRECTORY}/${TIMER_FILE}"
    done
  done

  # Make updated service files take effect
  sudo systemctl daemon-reload
fi

#
# Uninstall udev rules
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for DIRECTORY in \
    "${LIBFREENECT2_UDEV_DIRECTORY}" \
    "${OASIS_DATA_DIRECTORY}/oasis_drivers_py/udev" \
  ; do
    # Skip directories that don't exist
    if [ ! -d "${DIRECTORY}" ]; then
      echo "Doesn't exist: ${DIRECTORY}"
      continue
    fi

    # Process udev rules
    for UDEV_RULE in "${DIRECTORY}/"*.rules; do
      # Get filename
      FILE_NAME="$(basename -- "${UDEV_RULE}")"

      # Skip services that weren't installed
      if [ ! -f "${UDEV_RULE_DIRECTORY}/${FILE_NAME}" ]; then
        echo "Not installed: ${FILE_NAME}"
        continue
      fi

      echo "Uninstalling ${FILE_NAME}"

      sudo rm -f "${UDEV_RULE_DIRECTORY}/${FILE_NAME}"
    done
  done
fi

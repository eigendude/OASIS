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
# Hardware configuration
#

# Always enable drivers and hardware control
ENABLE_CONTROL=1
ENABLE_DRIVERS=1

# Machines for image processing
ENABLE_PERCEPTION=0
if \
  [ "${HOSTNAME}" = "bar" ] || \
  [ "${HOSTNAME}" = "cinder" ] || \
  [ "${HOSTNAME}" = "door" ] || \
  [ "${HOSTNAME}" = "kitchen" ] || \
  [ "${HOSTNAME}" = "station" ] \
; then
  ENABLE_PERCEPTION=1
fi

# Machines with a display
ENABLE_VISUALIZATION=0
if \
  [ "${HOSTNAME}" = "bar" ] || \
  [ "${HOSTNAME}" = "cinder" ] || \
  [ "${HOSTNAME}" = "door" ] || \
  [ "${HOSTNAME}" = "kitchen" ] || \
  [ "${HOSTNAME}" = "megapegasus" ] || \
  [ "${HOSTNAME}" = "nuc" ] || \
  [ "${HOSTNAME}" = "patio" ] \
; then
  ENABLE_VISUALIZATION=1
fi

#
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import Kodi paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

# Directory where systemd services are installed
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system/"

# Directory for udev rules
UDEV_RULE_DIRECTORY="/etc/udev/rules.d"

#
# Install systemd services
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for OASIS_PACKAGE in \
    $([ "${ENABLE_CONTROL}" = "0" ] || echo "oasis_control") \
    $([ "${ENABLE_DRIVERS}" = "0" ] || echo "oasis_drivers_py") \
    $([ "${ENABLE_PERCEPTION}" = "0" ] || echo "oasis_perception_py") \
    $([ "${ENABLE_VISUALIZATION}" = "0" ] || echo "oasis_visualization") \
  ; do
    # Skip packages that weren't build
    if [ ! -d "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd" ]; then
      echo "Skipping package ${OASIS_PACKAGE}"
      continue
    fi

    # Process systemd services
    for SYSTEMD_SERVICE in "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd/"*.service; do
      # Get filename
      FILE_NAME="$(basename -- "${SYSTEMD_SERVICE}")"

      # Skip oasis_visualization if Kodi wasn't built
      if [ "${OASIS_PACKAGE}" = "oasis_visualization" ] && [ ! -e "${KODI_BINARY}" ]; then
        echo "Skipping ${FILE_NAME}"
        continue
      fi

      echo "Installing ${FILE_NAME}"

      # Install systemd service
      sudo install -m 0644 "${SYSTEMD_SERVICE}" "${SYSTEMD_SERVICE_DIRECTORY}"

      # Now rewrite User= to the current user
      sudo sed -i "s|^User=.*|User=$(id -un)|" \
        "${SYSTEMD_SERVICE_DIRECTORY}/${FILE_NAME}"

      # Enable systemd servcie
      if [ ! -L "${SYSTEMD_SERVICE_DIRECTORY}/multi-user.target.wants/${FILE_NAME}" ]; then
        sudo ln -s \
          "${SYSTEMD_SERVICE_DIRECTORY}/${FILE_NAME}" \
          "${SYSTEMD_SERVICE_DIRECTORY}/multi-user.target.wants/${FILE_NAME}"
        fi
    done
  done

  # Make updated service files take effect
  sudo systemctl daemon-reload
fi

#
# Install udev rules
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

      echo "Installing ${FILE_NAME}"

      sudo cp "${UDEV_RULE}" "${UDEV_RULE_DIRECTORY}"
    done
  done

  # Reload udev rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
fi

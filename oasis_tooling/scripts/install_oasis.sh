#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
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

# Import Kodi paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

# Directory where systemd services are installed
SYSTEMD_SERVICE_DIRECTORY="/etc/systemd/system/"

# Directory for udev rules
UDEV_RULE_DIRECTORY="/etc/udev/rules.d"

#
# TODO: Hardware configuration
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
  [ "${HOSTNAME}" = "falcon" ] || \
  [ "${HOSTNAME}" = "hallway" ] || \
  [ "${HOSTNAME}" = "kitchen" ] || \
  [ "${HOSTNAME}" = "megapegasus" ] || \
  [ "${HOSTNAME}" = "station" ] \
; then
  ENABLE_PERCEPTION=1
fi

# Machines with a display
ENABLE_VISUALIZATION=0
if \
  [ "${HOSTNAME}" = "bar" ] || \
  [ "${HOSTNAME}" = "cinder" ] || \
  [ "${HOSTNAME}" = "desk" ] || \
  [ "${HOSTNAME}" = "door" ] || \
  [ "${HOSTNAME}" = "hallway" ] || \
  [ "${HOSTNAME}" = "kitchen" ] || \
  [ "${HOSTNAME}" = "megapegasus" ] || \
  [ "${HOSTNAME}" = "patio" ] || \
  [ "${HOSTNAME}" = "substation" ] \
; then
  ENABLE_VISUALIZATION=1
fi

# Get a list of enabled packages
ENABLED_PACKAGES=()
for OASIS_PACKAGE in \
    $([ "${ENABLE_CONTROL}" = "0" ] || echo "oasis_control") \
    $([ "${ENABLE_DRIVERS}" = "0" ] || echo "oasis_drivers_py") \
    $([ "${ENABLE_PERCEPTION}" = "0" ] || echo "oasis_perception_py") \
    $([ "${ENABLE_VISUALIZATION}" = "0" ] || echo "oasis_visualization") \
; do
  # Skip packages that weren't built
  if [ ! -d "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}" ]; then
    echo "Skipping package ${OASIS_PACKAGE}"
    continue
  fi

  # Skip oasis_visualization if Kodi wasn't built
  if [ "${OASIS_PACKAGE}" = "oasis_visualization" ] && [ ! -e "${KODI_BINARY}" ]; then
    echo "Skipping package ${OASIS_PACKAGE} (Kodi not built)"
    continue
  fi

  ENABLED_PACKAGES+=("${OASIS_PACKAGE}")
done

#
# Install camera info files
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Parent directory for ROS metadata
  ROS_DIRECTORY=~/".ros"
  CAMERA_INFO_LINK="${ROS_DIRECTORY}/camera_info"
  CAMERA_INFO_TARGET=""

  # Ensure ~/.ros exists for the symlink
  install -m 0755 -d "${ROS_DIRECTORY}"

  for OASIS_PACKAGE in "${ENABLED_PACKAGES[@]}"; do
    DIRECTORY="${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/camera_info"

    # Skip directories that don't exist
    if [ ! -d "${DIRECTORY}" ]; then
      continue
    fi

    CAMERA_INFO_TARGET="${DIRECTORY}"
    break
  done

  if [ -n "${CAMERA_INFO_TARGET}" ]; then
    echo "Linking camera info directory to ${CAMERA_INFO_TARGET}"

    # Remove any existing directory
    rm -rf "${CAMERA_INFO_LINK}"

    ln -sfn "${CAMERA_INFO_TARGET}" "${CAMERA_INFO_LINK}"
  fi
fi

#
# Install systemd services
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for OASIS_PACKAGE in "${ENABLED_PACKAGES[@]}"; do
    for SYSTEMD_SERVICE in "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd/"*.service; do
      SYSTEMD_TIMER="${SYSTEMD_SERVICE%.service}.timer"

      # Get service filename
      SERVICE_FILE="$(basename -- "${SYSTEMD_SERVICE}")"

      # Get timer filename
      TIMER_FILE="$(basename -- "${SYSTEMD_TIMER}")"

      # Install systemd service
      echo "Installing ${SERVICE_FILE}"
      sudo install -m 0644 "${SYSTEMD_SERVICE}" "${SYSTEMD_SERVICE_DIRECTORY}"

      # Installer timer, if present
      if [ -e "${SYSTEMD_TIMER}" ]; then
        echo "Installing ${TIMER_FILE}"
        sudo install -m 0644 "${SYSTEMD_TIMER}" "${SYSTEMD_SERVICE_DIRECTORY}"
      fi

      # Now rewrite User= to the current user
      sudo sed -i "s|^User=.*|User=$(id -un)|" \
        "${SYSTEMD_SERVICE_DIRECTORY}/${SERVICE_FILE}"
    done
  done

  # Make updated service files take effect
  sudo systemctl daemon-reload

  for OASIS_PACKAGE in "${ENABLED_PACKAGES[@]}"; do
    for SYSTEMD_SERVICE in "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd/"*.service; do
      SYSTEMD_TIMER="${SYSTEMD_SERVICE%.service}.timer"

      # Get service filename
      SERVICE_FILE="$(basename -- "${SYSTEMD_SERVICE}")"

      # Get timer filename
      TIMER_FILE="$(basename -- "${SYSTEMD_TIMER}")"

      # Enable systemd timer if present
      if [ -e "${SYSTEMD_TIMER}" ]; then
        echo "Enabling ${TIMER_FILE}"
        sudo systemctl enable "${TIMER_FILE}"
      else
        # Otherwise enable the service (unless it's a template unit)
        if [[ "${SERVICE_FILE}" == *@.service ]]; then
          echo "Skipping template unit ${SERVICE_FILE}"
        else
          echo "Enabling ${SERVICE_FILE}"
          sudo systemctl enable "${SERVICE_FILE}"
        fi
      fi
    done
  done
fi

#
# Install udev rules
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for DIRECTORY in \
    "${LIBFREENECT2_UDEV_DIRECTORY}" \
    "${OASIS_DATA_DIRECTORY}/oasis_avr/udev" \
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

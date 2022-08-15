#!/bin/bash
################################################################################
#
#  Copyright (C) 2022 Garrett Brown
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

#
# Install systemd services
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  for OASIS_PACKAGE in \
    oasis_control \
    oasis_drivers_py \
    oasis_perception \
    oasis_visualization \
  ; do
    # Skip packages that weren't build
    if [ ! -d "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd" ]; then
      echo "Skipping ${OASIS_PACKAGE} services"
      continue
    fi

    for SYSTEMD_SERVICE in "${OASIS_DATA_DIRECTORY}/${OASIS_PACKAGE}/systemd/"*.service; do
      # Get filename using bash syntax
      FILE_NAME="$(basename -- ${SYSTEMD_SERVICE})"

      # Skip oasis_visualization if Kodi wasn't built
      if [ "${OASIS_PACKAGE}" = "oasis_visualization" ] && [ ! -e "${KODI_BINARY}" ]; then
        echo "Skipping ${FILE_NAME}"
        continue
      fi

      echo "Installing ${FILE_NAME}"

      # Install systemd service
      sudo install -m 0644 "${SYSTEMD_SERVICE}" "${SYSTEMD_SERVICE_DIRECTORY}"

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

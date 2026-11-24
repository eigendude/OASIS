#!/bin/bash
################################################################################
#
#  Copyright (C) 2024-2026 Garrett Brown
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

################################################################################
# Environment paths and config
################################################################################

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

################################################################################
# Setup ROS 2 sources
################################################################################

# Remove any existing ROS apt sources added by older instructions
sudo rm -f \
  /etc/apt/sources.list.d/ros2-latest.list \
  /etc/apt/sources.list.d/ros2.list
sudo sed -i '/packages\.ros\.org\/ros2/d' /etc/apt/sources.list 2>/dev/null || true
sudo find /etc/apt/sources.list.d -name '*.list' -exec \
  sed -i '/packages\.ros\.org\/ros2/d' {} \; 2>/dev/null || true
sudo rm -f \
  /usr/share/keyrings/ros-archive-keyring.gpg \
  /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg

# Add the ROS 2 repository using the ros2-apt-source package
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  # Install required dependencies
  sudo apt install -y \
    curl \
    gnupg2

  TMP_DEB=$(mktemp --suffix .deb)

  download_package() {
    local package_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${1}_all.deb"
    curl -fsSL --retry 4 --retry-connrefused --connect-timeout 10 --max-time 60 \
      -o "${TMP_DEB}" "${package_url}"
  }

  if ! download_package "${CODENAME}"; then
    if [ "${CODENAME}" = "${ROS_APT_SOURCE_FALLBACK_CODENAME}" ]; then
      rm -f "${TMP_DEB}"
      exit 1
    fi

    echo "Download for ${CODENAME} failed, trying ${ROS_APT_SOURCE_FALLBACK_CODENAME}..." >&2
    if ! download_package "${ROS_APT_SOURCE_FALLBACK_CODENAME}"; then
      rm -f "${TMP_DEB}"
      exit 1
    fi
  fi

  sudo dpkg -i "${TMP_DEB}"
  rm -f "${TMP_DEB}"
fi

sudo apt update || :

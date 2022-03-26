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

# Import environment
source "${SCRIPT_DIR}/env_ros2_desktop.sh"
source "${SCRIPT_DIR}/env_kodi.sh"

# Import CMake paths
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Load ROS 2 Desktop environment
#

set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Load OASIS environment
#

set +o nounset
source "${STACK_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Download Kodi
#

if [ ! -f "${KODI_ARCHIVE_PATH}" ]; then
  echo "Downloading Kodi..."
  wget "${KODI_URL}" -O "${KODI_ARCHIVE_PATH}"
fi

#
# Extract Kodi
#

echo "Extracting Kodi..."
tar -zxf "${KODI_ARCHIVE_PATH}" --directory="${KODI_EXTRACT_DIR}"

#
# Configure Kodi
#

(
  echo "Configuring Kodi..."
  mkdir -p "${KODI_BUILD_DIR}"
  cd "${KODI_BUILD_DIR}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
    cmake \
      "${KODI_SOURCE_DIR}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${KODI_INSTALL_DIR}" \
      -DENABLE_INTERNAL_FLATBUFFERS=ON \
      -DAPP_RENDER_SYSTEM=${APP_RENDER_SYSTEM}
      #-DCORE_PLATFORM_NAME=x11
)

#
# Build Kodi
#

echo "Building Kodi..."
make -C "${KODI_BUILD_DIR}" -j$(getconf _NPROCESSORS_ONLN)

#
# Install Kodi
#

echo "Installing Kodi..."
make -C "${KODI_BUILD_DIR}" install

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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import Kodi paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

#
# Load OASIS environment
#

set +o nounset
source "${OASIS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Directory setup
#

# Create directories
mkdir -p "${KODI_DOWNLOAD_DIR}"
mkdir -p "${KODI_EXTRACT_DIR}"
mkdir -p "${KODI_BUILD_DIR}"

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
# Patch Kodi
#

echo "Patching Kodi..."
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${KODI_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/kodi/0001-depends-Remove-git-dependency.patch" \
  || :

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
      -DAPP_RENDER_SYSTEM=${APP_RENDER_SYSTEM} \
      -DCORE_PLATFORM_NAME="x11 gbm$([ "${ENABLE_WAYLAND}" = "OFF" ] || echo " wayland")" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${KODI_INSTALL_DIR}" \
      -DENABLE_CEC=OFF \
      -DENABLE_INTERNAL_FLATBUFFERS=ON \
      -DENABLE_INTERNAL_SPDLOG=ON \
      -DENABLE_LLD=${ENABLE_LLD} \
      -DENABLE_TESTING=OFF \
)

#
# Build Kodi
#

echo "Building Kodi..."
make \
  -C "${KODI_BUILD_DIR}" \
  -j$(getconf _NPROCESSORS_ONLN) \

#
# Install Kodi
#

echo "Installing Kodi..."
make -C "${KODI_BUILD_DIR}" install

#
# Build add-ons
#

echo "Building add-ons..."
make \
  -C "${KODI_DEPENDS}/target/binary-addons" \
  -j$(getconf _NPROCESSORS_ONLN) \
  ADDONS="peripheral.joystick" \
  PREFIX="${KODI_INSTALL_DIR}" \

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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import OpenCV paths and config
source "${SCRIPT_DIR}/env_cv.sh"

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

# Remove existing src directory if it exists
if [ -d "${KODI_SOURCE_DIR}" ]; then
  echo "Removing existing src directory..."
  rm -rf "${KODI_SOURCE_DIR}"
fi

# Create directories
mkdir -p "${KODI_DOWNLOAD_DIR}"
mkdir -p "${KODI_SOURCE_DIR}"
mkdir -p "${KODI_BUILD_DIR}"
mkdir -p "${KODI_DEPENDS_DIR}"

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
tar -zxf "${KODI_ARCHIVE_PATH}" --strip-components=1 --directory="${KODI_SOURCE_DIR}"

#
# Patch Kodi
#

echo "Patching Kodi..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${KODI_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/kodi/0001-depends-Remove-git-dependency.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${KODI_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/kodi/0001-temp-Wait-until-Wayland-display-is-available.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${KODI_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/kodi/0002-temp-Wayland-Disable-window-decorations.patch"
if [ -f "${CONFIG_DIRECTORY}/kodi/0003-temp-Force-${HOSTNAME}-dimensions.patch" ]; then
  patch \
    -p1 \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${KODI_SOURCE_DIR}" \
    < "${CONFIG_DIRECTORY}/kodi/0003-temp-Force-${HOSTNAME}-dimensions.patch"
fi

#
# Configure Kodi
#

(
  echo "Configuring Kodi..."
  mkdir -p "${KODI_BUILD_DIR}"
  cd "${KODI_BUILD_DIR}"
  PATH="${CMAKE_BIN_DIRECTORY}:${PATH}" \
  PKG_CONFIG_PATH="${KODI_DEPENDS_DIR}/${KODI_DEPENDS_TARGET}/lib/pkgconfig" \
    cmake \
      "${KODI_SOURCE_DIR}" \
      -DAPP_RENDER_SYSTEM=${APP_RENDER_SYSTEM} \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="${KODI_INSTALL_DIR}" \
      -DCORE_PLATFORM_NAME="wayland" \
      -DENABLE_CEC=OFF \
      -DENABLE_INTERNAL_FFMPEG=ON \
      -DENABLE_LLD=${ENABLE_LLD} \
      -DENABLE_ROS2=ON \
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
  -C "${KODI_DEPENDS_SRC}/target/binary-addons" \
  -j$(getconf _NPROCESSORS_ONLN) \
  ADDONS="^(peripheral.joystick|screensaver.matrixtrails)$" \
  EXTRA_CMAKE_ARGS="-DAPP_RENDER_SYSTEM=gl" \
  PREFIX="${KODI_INSTALL_DIR}" \

#
# Install additional add-ons
#

WEATHER_ICONS_ADDON="resource.images.weathericons.hd.animated"
WEATHER_ICONS_URL="https://mirrors.kodi.tv/addons/piers/${WEATHER_ICONS_ADDON}/${WEATHER_ICONS_ADDON}-1.0.0.zip"
WEATHER_ICONS_ZIP="${KODI_DOWNLOAD_DIR}/${WEATHER_ICONS_ADDON}-1.0.0.zip"
WEATHER_ICONS_ADDON_DIR="${KODI_INSTALL_DIR}/share/kodi/addons"
WEATHER_ICONS_INSTALL_DIR="${WEATHER_ICONS_ADDON_DIR}/${WEATHER_ICONS_ADDON}"
WEATHER_ICONS_ADDON_XML="${WEATHER_ICONS_INSTALL_DIR}/addon.xml"

# Ensure directories exist
mkdir -p "${WEATHER_ICONS_ADDON_DIR}"
mkdir -p "${WEATHER_ICONS_INSTALL_DIR}"

if [ ! -f "${WEATHER_ICONS_ZIP}" ]; then
  echo "Downloading ${WEATHER_ICONS_ADDON} add-on..."
  wget "${WEATHER_ICONS_URL}" -O "${WEATHER_ICONS_ZIP}"
fi

if [ ! -f "${WEATHER_ICONS_ADDON_XML}" ]; then
  echo "Installing ${WEATHER_ICONS_ADDON} add-on..."
  unzip -o "${WEATHER_ICONS_ZIP}" -d "${WEATHER_ICONS_ADDON_DIR}"
else
  echo "${WEATHER_ICONS_ADDON} add-on already installed, skipping..."
fi

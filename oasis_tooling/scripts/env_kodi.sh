#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2022 Garrett Brown
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
# Environment configuration
#

CODENAME="$(lsb_release --codename | cut -f2)"

#
# Hardware configuration
#

# Machines with dual displays
ENABLE_DUAL_DISPLAYS=0
if \
  [ "${HOSTNAME}" = "nuc" ] \
; then
  ENABLE_DUAL_DISPLAYS=1
fi

#
# Kodi configuration
#

# Version
if [ "${ENABLE_DUAL_DISPLAYS}" = "1" ]; then
  KODI_VERSION="68cda663e4688880c892cbce44cf620884efb9ad"
else
  KODI_VERSION="993b61c9d0c0efc6b74c2c67a6a398c411c51eed"
fi

# URL
KODI_URL="https://github.com/garbear/xbmc/archive/${KODI_VERSION}.tar.gz"

#
# Build environment
#

APP_RENDER_SYSTEM=gles

# Enable LLD if available
ENABLE_LLD="$([ -n "$(apt-cache search --names-only '^lld$')" ] && echo "ON" || echo "OFF")"

# Enable Wayland if the waylandpp-dev package is found
ENABLE_WAYLAND="$([ -n "$(apt-cache search --names-only '^waylandpp-dev$')" ] && echo "ON" || echo "OFF")"

# Enable internal spdlog on Ubuntu 18.04
if [ "${CODENAME}" = "bionic" ]; then
  ENABLE_INTERNAL_FMT=ON
  ENABLE_INTERNAL_SPDLOG=ON
else
  ENABLE_INTERNAL_FMT=OFF
  ENABLE_INTERNAL_SPDLOG=OFF
fi

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Directory and path definitions
#

# Subdirectory for Kodi build files
KODI_DIRECTORY="${BUILD_DIRECTORY}/kodi-${ROS2_DISTRO}"

# Define top-level directories for Kodi
KODI_DOWNLOAD_DIR="${KODI_DIRECTORY}/downloads"
KODI_EXTRACT_DIR="${KODI_DIRECTORY}/src"
KODI_SOURCE_DIR="${KODI_DIRECTORY}/src/xbmc-${KODI_VERSION}"
KODI_BUILD_DIR="${KODI_DIRECTORY}/build/kodi-${KODI_VERSION}"
KODI_INSTALL_DIR="${KODI_DIRECTORY}/install"

# Installed Kodi binaries (for external use of Kodi)
KODI_BIN_DIRECTORY="${KODI_INSTALL_DIR}/bin"

# Kodi depends directory
KODI_DEPENDS="${KODI_SOURCE_DIR}/tools/depends"

# Define paths
KODI_ARCHIVE_PATH="${KODI_DOWNLOAD_DIR}/kodi-${KODI_VERSION}.tar.gz"

# Installed Kodi executable
KODI_BINARY="${KODI_BIN_DIRECTORY}/kodi"

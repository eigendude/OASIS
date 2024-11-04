#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
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
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Kodi configuration
#

# Version
if [ "${HOSTNAME}" == "lenovo" ]; then
  KODI_VERSION="385c2688336a609be5a822d2043405508d41e8ce"
elif [ "${HOSTNAME}" == "patio" ]; then
  KODI_VERSION="79494ee8546e370fdd3663d3b93942baa54b7b82"
else
  KODI_VERSION="7e004b4011203af97cdf3d6bfcc587d2b8382184"
fi

# URL
KODI_URL="https://github.com/garbear/xbmc/archive/${KODI_VERSION}.tar.gz"

# Target architecture and build config
KODI_DEPENDS_TARGET=${PLATFORM_ARCH}-linux-gnu-release

#
# Build environment
#

# Render system
if [[ ${PLATFORM_ARCH} == x86_64 ]]; then
  APP_RENDER_SYSTEM=gl
else
  APP_RENDER_SYSTEM=gles
fi

# Enable LLD if available
ENABLE_LLD="$([ -n "$(apt-cache search --names-only '^lld$')" ] && echo "ON" || echo "OFF")"

# Enable Wayland if the waylandpp-dev package is found
ENABLE_WAYLAND="$([ -n "$(apt-cache search --names-only '^waylandpp-dev$')" ] && echo "ON" || echo "OFF")"

# Enable internal spdlog on Ubuntu 18.04 and Ubuntu 22.04
if [ "${CODENAME}" = "bionic" ] || [ "${CODENAME}" = "jammy" ]; then
  ENABLE_INTERNAL_FMT=ON
  ENABLE_INTERNAL_SPDLOG=ON
else
  ENABLE_INTERNAL_FMT=OFF
  ENABLE_INTERNAL_SPDLOG=OFF
fi

#
# Directory and path definitions
#

# Subdirectory for Kodi build files
KODI_DIRECTORY="${BUILD_DIRECTORY}/kodi-${ROS2_DISTRO}"

# Define top-level directories for Kodi
KODI_DOWNLOAD_DIR="${KODI_DIRECTORY}/downloads"
KODI_SOURCE_DIR="${KODI_DIRECTORY}/src"
KODI_DEPENDS_DIR="${KODI_DIRECTORY}/depends"
KODI_BUILD_DIR="${KODI_DIRECTORY}/build/kodi-${KODI_VERSION}"
KODI_INSTALL_DIR="${KODI_DIRECTORY}/install"

# Installed Kodi binaries (for external use of Kodi)
KODI_BIN_DIRECTORY="${KODI_INSTALL_DIR}/bin"

# Kodi depends directory
KODI_DEPENDS_SRC="${KODI_SOURCE_DIR}/tools/depends"

# Define paths
KODI_ARCHIVE_PATH="${KODI_DOWNLOAD_DIR}/kodi-${KODI_VERSION}.tar.gz"

# Installed Kodi executable
KODI_BINARY="${KODI_BIN_DIRECTORY}/kodi"

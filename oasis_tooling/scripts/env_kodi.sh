#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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

# Shell-source the key/value pairs defined in /etc/os-release
source "/etc/os-release"

# Debian/Ubuntu expose the codename as VERSION_CODENAME
CODENAME="${VERSION_CODENAME}"

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
KODI_VERSION="a2e2584a8595d1420cdec2c6a95ef6f2bf9487e4"

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

# Enable LLD
ENABLE_LLD="ON"

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

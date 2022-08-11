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
# Kodi configuration
#

# Version
KODI_VERSION="9c489418960581f0aa499d5a21953d39bd862dbb"

# URL
KODI_URL="https://github.com/garbear/xbmc/archive/${KODI_VERSION}.tar.gz"

#
# Build environment
#

APP_RENDER_SYSTEM=gles

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

# Import Python paths and config
source "${SCRIPT_DIR}/env_python.sh"

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

# Import OASIS dependency paths and config
source "${SCRIPT_DIR}/env_oasis_deps.sh"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Directory and path definitions
#

# Subdirectory for KODI build files
KODI_DIRECTORY="${BUILD_DIRECTORY}/kodi"

# Define top-level directories for Kodi
KODI_DOWNLOAD_DIR="${KODI_DIRECTORY}/downloads"
KODI_EXTRACT_DIR="${KODI_DIRECTORY}/src"
KODI_SOURCE_DIR="${KODI_DIRECTORY}/src/xbmc-${KODI_VERSION}"
KODI_BUILD_DIR="${KODI_DIRECTORY}/build/kodi-${KODI_VERSION}"
KODI_INSTALL_DIR="${KODI_DIRECTORY}/install"

# Installed Kodi binaries (for external use of Kodi)
KODI_BIN_DIRECTORY="${KODI_INSTALL_DIR}/bin"

# Define paths
KODI_ARCHIVE_PATH="${KODI_DOWNLOAD_DIR}/kodi-${KODI_VERSION}.tar.gz"

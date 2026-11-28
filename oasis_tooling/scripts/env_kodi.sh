#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Debian/Ubuntu expose the codename as VERSION_CODENAME
  source "/etc/os-release"
  CODENAME="${VERSION_CODENAME}"
fi

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

#
# Kodi configuration
#

# Version
KODI_VERSION="65bfc9a937b8128f698f079ef57a3d8a0c81b6bf"

# URL
KODI_URL="https://github.com/garbear/xbmc"

# Target architecture and build config
if [[ "${OSTYPE}" == "darwin"* ]]; then
  case "$(uname -m)" in
    arm64) KODI_DEPENDS_HOST="aarch64-apple-darwin" ;;
    x86_64) KODI_DEPENDS_HOST="x86_64-apple-darwin" ;;
    *)
      echo "Unsupported macOS architecture: $(uname -m)" >&2
      return 1
      ;;
  esac
else
  KODI_DEPENDS_TARGET=${PLATFORM_ARCH}-linux-gnu-release
fi

#
# Build environment
#

# Render system
if [[ "${OSTYPE}" == "darwin"* ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
  APP_RENDER_SYSTEM=gl
else
  APP_RENDER_SYSTEM=gles
fi

# Enable LLD
ENABLE_LLD="ON"
if [[ "${OSTYPE}" == "darwin"* ]]; then
  ENABLE_LLD="OFF"
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

# Installed Kodi executable
KODI_BINARY="${KODI_BIN_DIRECTORY}/kodi"
if [[ "${OSTYPE}" == "darwin"* ]]; then
  KODI_BIN_DIRECTORY="${KODI_INSTALL_DIR}/Kodi.app/Contents/MacOS"
  KODI_BINARY="${KODI_BIN_DIRECTORY}/Kodi"
fi

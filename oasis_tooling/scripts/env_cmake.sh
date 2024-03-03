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
# CMake configuration
#

# Version
CMAKE_VERSION="3.28.3"

# URL
CMAKE_URL="https://codeload.github.com/Kitware/CMake/tar.gz/refs/tags/v${CMAKE_VERSION}"

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

#
# Directory and path definitions
#

# Subdirectory for CMAKE build files
CMAKE_DIRECTORY="${BUILD_DIRECTORY}/cmake"

# Define top-level directories for CMake
CMAKE_DOWNLOAD_DIR="${CMAKE_DIRECTORY}/downloads"
CMAKE_EXTRACT_DIR="${CMAKE_DIRECTORY}/src"
CMAKE_SOURCE_DIR="${CMAKE_DIRECTORY}/src/CMake-${CMAKE_VERSION}"
CMAKE_BUILD_DIR="${CMAKE_DIRECTORY}/build/cmake-${CMAKE_VERSION}"
CMAKE_INSTALL_DIR="${CMAKE_DIRECTORY}/install"

# Installed CMake binaries (for external use of CMake)
CMAKE_BIN_DIRECTORY="${CMAKE_INSTALL_DIR}/bin"

# Define paths
CMAKE_ARCHIVE_PATH="${CMAKE_DOWNLOAD_DIR}/cmake-${CMAKE_VERSION}.tar.gz"
CMAKE_LISTS_PATH="${CMAKE_SOURCE_DIR}/CMakeLists.txt"
CMAKE_MAKEFILE_PATH="${CMAKE_BUILD_DIR}/Makefile"

#
# System path setup
#

# Add CMake to system path
export PATH="${CMAKE_BIN_DIRECTORY}:${PATH}"

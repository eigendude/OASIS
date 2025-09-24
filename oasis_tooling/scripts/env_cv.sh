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
# OpenCV configuration
#

# Version
OPENCV_VERSION="4.12.0"

# URLS
OPENCV_URL="https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz"
OPENCV_CONTRIB_URL="https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.tar.gz"

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

#
# Build environment
#

#
# Directory and path definitions
#

# Subdirectory for OpenCV build files
OPENCV_DIRECTORY="${BUILD_DIRECTORY}/opencv"

# Define top-level directories for OpenCV
OPENCV_DOWNLOAD_DIR="${OPENCV_DIRECTORY}/downloads"
OPENCV_EXTRACT_DIR="${OPENCV_DIRECTORY}/src"
OPENCV_SOURCE_DIR="${OPENCV_DIRECTORY}/src/opencv-${OPENCV_VERSION}"
OPENCV_CONTRIB_SOURCE_DIR="${OPENCV_DIRECTORY}/src/opencv_contrib-${OPENCV_VERSION}"
OPENCV_BUILD_DIR="${OPENCV_DIRECTORY}/build/opencv-${OPENCV_VERSION}"
OPENCV_CONTRIB_BUILD_DIR="${OPENCV_DIRECTORY}/build/opencv_contrib-${OPENCV_VERSION}"
OPENCV_INSTALL_DIR="${OPENCV_DIRECTORY}/install"

# Define paths
OPENCV_ARCHIVE_PATH="${OPENCV_DOWNLOAD_DIR}/opencv-${OPENCV_VERSION}.tar.gz"
OPENCV_CONTRIB_ARCHIVE_PATH="${OPENCV_DOWNLOAD_DIR}/opencv_contrib-${OPENCV_VERSION}.tar.gz"

# Expose the custom OpenCV build to CMake and downstream packages
export OpenCV_DIR="${OPENCV_INSTALL_DIR}/lib/cmake/opencv4"
export OpenCV_ROOT="${OPENCV_INSTALL_DIR}"

if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="${OPENCV_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
else
  export CMAKE_PREFIX_PATH="${OPENCV_INSTALL_DIR}"
fi

# Make OpenCV's shared libraries available at runtime
OPENCV_LIBRARY_DIRECTORY="${OPENCV_INSTALL_DIR}/lib"

if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  export LD_LIBRARY_PATH="${OPENCV_LIBRARY_DIRECTORY}:${LD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="${OPENCV_LIBRARY_DIRECTORY}"
fi

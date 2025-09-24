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

# URL
OPENCV_URL="https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz"

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
OPENCV_SOURCE_DIR="${OPENCV_EXTRACT_DIR}/opencv-${OPENCV_VERSION}"
OPENCV_DEPENDS_DIR="${OPENCV_DIRECTORY}/depends"
OPENCV_BUILD_DIR="${OPENCV_DIRECTORY}/build/opencv-${OPENCV_VERSION}"
OPENCV_INSTALL_DIR="${OPENCV_DIRECTORY}/install"

# Installed OpenCV binaries (for external use of OpenCV)
OPENCV_BIN_DIRECTORY="${OPENCV_INSTALL_DIR}/bin"

# OpenCV depends directory
OPENCV_DEPENDS_SRC="${OPENCV_SOURCE_DIR}/tools/depends"

# Define paths
OPENCV_ARCHIVE_PATH="${OPENCV_DOWNLOAD_DIR}/opencv-${OPENCV_VERSION}.tar.gz"

#!/bin/bash
################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
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
OPENCV_VERSION="4.14.0"

# Extract major version (everything before the first '.')
OPENCV_MAJOR_VERSION="${OPENCV_VERSION%%.*}"

# URLS
OPENCV_URL="https://github.com/opencv/opencv.git"
OPENCV_CONTRIB_URL="https://github.com/opencv/opencv_contrib.git"

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Import the shared Python environment
source "${SCRIPT_DIR}/env_python.sh"

#
# Build environment
#

#
# Directory and path definitions
#

# Subdirectory for OpenCV build files
OPENCV_DIRECTORY="${BUILD_DIRECTORY}/cv"

# Define top-level directories for OpenCV
OPENCV_SOURCE_DIR="${OPENCV_DIRECTORY}/src/opencv-${OPENCV_VERSION}"
OPENCV_CONTRIB_SOURCE_DIR="${OPENCV_DIRECTORY}/src/opencv_contrib-${OPENCV_VERSION}"
OPENCV_BUILD_DIR="${OPENCV_DIRECTORY}/build/opencv-${OPENCV_VERSION}"
OPENCV_CONTRIB_BUILD_DIR="${OPENCV_DIRECTORY}/build/opencv_contrib-${OPENCV_VERSION}"
OPENCV_INSTALL_DIR="${OPENCV_DIRECTORY}/install"

# Expose the custom OpenCV build to CMake and downstream packages
export OpenCV_DIR="${OPENCV_INSTALL_DIR}/lib/cmake/opencv${OPENCV_MAJOR_VERSION}"
export OpenCV_ROOT="${OPENCV_INSTALL_DIR}"

if [ -n "${CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="${OPENCV_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
else
  export CMAKE_PREFIX_PATH="${OPENCV_INSTALL_DIR}"
fi

# Make OpenCV's shared libraries available at runtime
OPENCV_LIBRARY_DIR="${OPENCV_INSTALL_DIR}/lib"

if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  export LD_LIBRARY_PATH="${OPENCV_LIBRARY_DIR}:${LD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="${OPENCV_LIBRARY_DIR}"
fi

# Make OpenCV's source-built Python bindings available at runtime
OPENCV_PYTHON_INSTALL_DIR="${OPENCV_PYTHON_INSTALL_DIR:-${OPENCV_INSTALL_DIR}/python}"
export OPENCV_PYTHON_INSTALL_DIR

if [ -n "${PYTHONPATH:-}" ]; then
  export PYTHONPATH="${OPENCV_PYTHON_INSTALL_DIR}:${PYTHONPATH}"
else
  export PYTHONPATH="${OPENCV_PYTHON_INSTALL_DIR}"
fi

#!/bin/bash
################################################################################
#
#  Copyright (C) 2021 Garrett Brown
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
# Integration script to build CMake
#
# A check is made to see if the system CMake can be used to compile a ROS 2
# desktop install, and this script exits if so.
#
# This script is included because ROS 2 requires a newer version of CMake, and
# Ubuntu 18.04 has an older version (3.10) in its package manager.
#
# Dependencies are automatically installed by the script.
#

# Version
CMAKE_VERSION="3.21.4"

# URL
CMAKE_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz"

#
# Check for system CMake with sufficient version
#

# We only need to build CMake if the system version is too old
CMAKE_MIN_REQUIRED=16

# Read system CMake version
EXISTING_CMAKE_VERSION=$(cmake --version | head -n 1 | awk '{print $$3}')
EXISTING_CMAKE_VERSION_MAJOR=$(echo ${EXISTING_CMAKE_VERSION} | cut -f1 -d.)
EXISTING_CMAKE_VERSION_MINOR=$(echo ${EXISTING_CMAKE_VERSION} | cut -f2 -d.)

# Decide if we should build CMake
if [ ${EXISTING_CMAKE_VERSION_MINOR} -lt ${CMAKE_MIN_REQUIRED} ]; then
  BUILD_CMAKE=1
else
  BUILD_CMAKE=0
fi

# If we don't need to build CMake, exit now
if [ ${BUILD_CMAKE} -eq 0 ]; then
  echo "Using system CMake version ${EXISTING_CMAKE_VERSION_MAJOR}.${EXISTING_CMAKE_VERSION_MINOR}"
  exit 0
else
  echo "Building CMake ${CMAKE_VERSION}"
  echo
fi

#
# Directory and path definitions
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Directory of the ROS 2 package
PACKAGE_DIRECTORY=${SCRIPT_DIR}/..

# Directory of the OASIS repo
REPO_DIRECTORY=${PACKAGE_DIRECTORY}/..

# Subdirectory for ROS build files
BUILD_DIRECTORY=${REPO_DIRECTORY}/ros-ws

# Subdirectory for CMAKE build files
CMAKE_DIRECTORY=${BUILD_DIRECTORY}/cmake

# Define directories
DOWNLOAD_DIR="${CMAKE_DIRECTORY}/downloads"
EXTRACT_DIR="${CMAKE_DIRECTORY}/src"
BUILD_DIR="${CMAKE_DIRECTORY}/build"
INSTALL_DIR="${CMAKE_DIRECTORY}/install"

# Define paths
CMAKE_ARCHIVE_PATH="${DOWNLOAD_DIR}/cmake-${CMAKE_VERSION}.tar.gz"
CMAKE_SOURCE_DIR="${EXTRACT_DIR}/cmake-${CMAKE_VERSION}"
CMAKE_LISTS_PATH="${CMAKE_SOURCE_DIR}/CMakeLists.txt"
CMAKE_BUILD_DIR="${BUILD_DIR}/cmake-${CMAKE_VERSION}"
CMAKE_MAKEFILE_PATH="${CMAKE_BUILD_DIR}/Makefile"

# Create directories
mkdir -p "${DOWNLOAD_DIR}"
mkdir -p "${EXTRACT_DIR}"
mkdir -p "${BUILD_DIR}"

# Exclude build directory from rosdep installs
touch "${BUILD_DIRECTORY}/CATKIN_IGNORE"

# Exclude build directory from Colcon builds
touch "${BUILD_DIRECTORY}/COLCON_IGNORE"

#
# Install dependencies
#

sudo apt update
sudo apt install -y \
  ccache \
  cmake \
  libssl-dev \
  make \
  tar \
  wget \

#
# Download CMake
#

if [ ! -f "${CMAKE_ARCHIVE_PATH}" ]; then
  echo "Downloading CMake..."
  wget "${CMAKE_URL}" -O "${CMAKE_ARCHIVE_PATH}"
fi

#
# Extract CMake
#

if [ ! -f "${CMAKE_LISTS_PATH}" ]; then
  echo "Extracting CMake..."
  tar -zxf "${CMAKE_ARCHIVE_PATH}" --directory="${EXTRACT_DIR}"
fi

#
# Configure CMake
#
# Takes about 5 minutes on a slow netbook, 10 minutes on a BeagleBone AI
#

if [ ! -f "${CMAKE_MAKEFILE_PATH}" ]; then
  (
    echo "Configuring CMake..."
    mkdir -p "${CMAKE_BUILD_DIR}"
    cd "${CMAKE_BUILD_DIR}"
    cmake \
      "${CMAKE_SOURCE_DIR}" \
      -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
      -DCMAKE_BUILD_PARALLEL_LEVEL="$(getconf _NPROCESSORS_ONLN)" \
      $(! command -v ccache &> /dev/null || echo "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache")
  )
fi

#
# Build CMake
#
# Takes about 90 minutes on a slow netbook, so grab a coffee.
#

echo "Building CMake..."
make -C "${CMAKE_BUILD_DIR}" -j$(getconf _NPROCESSORS_ONLN)

#
# Install CMake
#

echo "Installing CMake..."
make -C "${CMAKE_BUILD_DIR}" install

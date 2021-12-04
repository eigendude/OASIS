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
# Directory and path definitions
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Directory of the ROS 2 package
PACKAGE_DIRECTORY="${SCRIPT_DIR}/.."

# Directory of package configuration
CONFIG_DIRECTORY="${PACKAGE_DIRECTORY}/config"

# Directory of the ROS 2 stack (OASIS repo)
STACK_DIRECTORY="${PACKAGE_DIRECTORY}/.."

# Subdirectory for ROS build files
BUILD_DIRECTORY="${STACK_DIRECTORY}/ros-ws"

# Create directories
mkdir -p "${BUILD_DIRECTORY}"

# Exclude build directory from rosdep installs
touch "${BUILD_DIRECTORY}/CATKIN_IGNORE"

# Exclude build directory from Colcon builds
touch "${BUILD_DIRECTORY}/COLCON_IGNORE"

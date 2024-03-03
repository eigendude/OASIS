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
# ROS 2 configuration
#

# Define the ROS distro to use
ROS2_DISTRO=iron

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import Python paths and config
source "${SCRIPT_DIR}/env_python.sh"

# Ament Python package install prefix
AMENT_PYTHON_PKG_PREFIX="${PYTHON_PKG_PREFIX/site-packages/dist-packages}"

#
# Directory definitions
#

# Directory for ROS 2 Desktop build files
ROS2_DESKTOP_DIRECTORY="${BUILD_DIRECTORY}/ros2-desktop-${ROS2_DISTRO}"

# Directory for ROS 2 Desktop sources
ROS2_SOURCE_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/src"

# Directory for ROS 2 Desktop installed files
ROS2_INSTALL_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/install"

# Directory for ROS 2 Python packages
ROS2_PYTHON_PKG_DIRECTORY="${ROS2_INSTALL_DIRECTORY}/${PYTHON_PKG_PREFIX}"

# Ament installs into "local" subdirectory
AMENT_INSTALL_DIRECTORY="${ROS2_INSTALL_DIRECTORY}/local"

# Directory for Ament Python packages
AMENT_PYTHON_PKG_DIRECTORY="${ROS2_INSTALL_DIRECTORY}/${AMENT_PYTHON_PKG_PREFIX}"

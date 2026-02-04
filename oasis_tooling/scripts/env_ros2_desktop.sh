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
# ROS 2 configuration
#

# Define the ROS distro to use
ROS2_DISTRO=kilted

#
# Environment paths and config
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import common paths and config
source "${SCRIPT_DIR}/env_common.sh"

#
# Directory definitions
#

# Directory for ROS 2 Desktop build files
ROS2_DESKTOP_DIRECTORY="${BUILD_DIRECTORY}/ros2-desktop-${ROS2_DISTRO}"

# Directory for ROS 2 Desktop sources
ROS2_SOURCE_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/src"

# Directory for ROS 2 Desktop installed files
ROS2_INSTALL_DIRECTORY="${ROS2_DESKTOP_DIRECTORY}/install"

# Ament installs into "local" subdirectory
AMENT_INSTALL_DIRECTORY="${ROS2_INSTALL_DIRECTORY}/local"

#
# CMake environment
#

# Packages that install into ${ROS2_INSTALL_DIRECTORY}/opt/<pkg>.
# These directories may not exist yet when this script is sourced, and that's
# fine. CMake will ignore missing prefixes until they appear later in the build.
ROS2_OPT_PACKAGES=(
  gz_cmake_vendor
  gz_math_vendor
  gz_utils_vendor
)

ROS2_CMAKE_PREFIX_PATH="${AMENT_INSTALL_DIRECTORY};${ROS2_INSTALL_DIRECTORY}"

for pkg in "${ROS2_OPT_PACKAGES[@]}"; do
  ROS2_CMAKE_PREFIX_PATH="${ROS2_INSTALL_DIRECTORY}/opt/${pkg};${ROS2_CMAKE_PREFIX_PATH}"
done

#
# Shared library environment
#

# Ensure runtime-built tools can find freshly-built shared libs during colcon
# builds (e.g. cyclonedds runs idlc which links against libiceoryx_posh.so).
ROS_LIBDIR="${ROS2_INSTALL_DIRECTORY}/lib"
AMENT_LIBDIR="${AMENT_INSTALL_DIRECTORY}/lib"

export LD_LIBRARY_PATH="${AMENT_LIBDIR}:${ROS_LIBDIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

#
# Python environment
#

# Ensure ROS Python packages are importable in all CI environments. Some runners
# end up with PYTHONPATH missing the ROS install site-packages when CMake runs
# execute_process() during colcon builds.
PYTHON_VERSION="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"

ROS_SITE_PACKAGES="${ROS2_INSTALL_DIRECTORY}/lib/python${PYTHON_VERSION}/site-packages"
AMENT_SITE_PACKAGES="${AMENT_INSTALL_DIRECTORY}/lib/python${PYTHON_VERSION}/site-packages"

export PYTHONPATH="${AMENT_SITE_PACKAGES}:${ROS_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

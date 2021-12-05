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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import environment
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

#
# Load ROS 2 environment
#

# Load ROS 2 Desktop
set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Download OASIS dependency source code
#

vcs import "${OASIS_SOURCE_DIRECTORY}" < "${PACKAGE_DIRECTORY}/config/depends.repos"

#
# Install rosdeps packages
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Download dependencies
echo "Downloading dependency source code..."

# Install dependency rosdeps
rosdep install \
  --from-paths "${OASIS_SOURCE_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS2_DISTRO} \
  -y

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache

# TODO: image_transport needs libtinyxml2-dev indirectly
dpkg -s libtinyxml2-dev >/dev/null || sudo apt install -y libtinyxml2-dev

# Needed for OpenNI
for package in \
    default-jdk \
    libudev-dev \
    libusb-1.0-0-dev \
; do
  dpkg -s ${package} >/dev/null || sudo apt install -y ${package}
done

# Needed for libfreenect2
for package in \
    libturbojpeg0-dev \
    libusb-1.0-0-dev \
    ocl-icd-opencl-dev \
; do
  dpkg -s ${package} >/dev/null || sudo apt install -y ${package}
done

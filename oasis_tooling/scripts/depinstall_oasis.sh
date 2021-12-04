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
# Load ROS 2 Desktop environment
#

set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Install OASIS rosdeps
#

# Install rosdep
[ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

# Update rosdep
rosdep update

# Install OASIS rosdeps
rosdep install \
  --from-paths "${STACK_DIRECTORY}" \
  --ignore-src \
  --rosdistro ${ROS2_DISTRO} \
  -y

# Add ccache support
dpkg -s ccache >/dev/null || sudo apt install -y ccache

# TODO: image_transport needs libtinyxml2-dev indirectly
dpkg -s libtinyxml2-dev >/dev/null || sudo apt install -y libtinyxml2-dev

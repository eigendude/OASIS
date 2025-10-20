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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OASIS paths and config
source "${SCRIPT_DIR}/env_oasis.sh"

# rosdep keys to ignore
ROSDEP_IGNORE_KEYS=" \
  libopencv-dev \
"

#
# Load OASIS dependency environment
#

set +o nounset
source "${OASIS_DEPENDS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

#
# Setup ROS 2 sources
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  "${SCRIPT_DIR}/setup_ros2_desktop.sh"
fi

#
# Install build dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Packages to install via apt
  APT_PACKAGES=(
    # General development tools and ROS tools
    build-essential
    ccache
    cmake
    git
    ros-dev-tools
  )

  if [[ -d "${STACK_DIRECTORY}/oasis_msgs" ]] && \
     [[ ! -f "${STACK_DIRECTORY}/oasis_msgs/COLCON_IGNORE" ]]; then
    # Needed for ROS message generation
    APT_PACKAGES+=(
      python3-lark
      python3-numpy
    )
  fi

  if [[ -d "${STACK_DIRECTORY}/oasis_perception_cpp" ]] && \
     [[ ! -f "${STACK_DIRECTORY}/oasis_perception_cpp/COLCON_IGNORE" ]]; then
    APT_PACKAGES+=(
      # Needed by image_transport and plugins
      libconsole-bridge-dev
      libspdlog-dev
      libtinyxml2-dev

      # Needed to link MediaPipe, as its Bazel build system doesn't export
      # dependencies
      libegl-dev
      libgles-dev
      libgoogle-glog-dev
      libprotobuf-dev

      # Needed for custom OpenCV build
      libavcodec-dev
      libavformat-dev
      libavif-dev
      libavutil-dev
      libgstreamer-plugins-base1.0-dev
      libgstreamer1.0-dev
      libopenblas-dev
      libopenexr-dev
      libswscale-dev
    )
  fi

  sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"
fi

#
# Install OASIS rosdeps
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install rosdep
  [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

  # Update rosdep
  rosdep update

  # Install OASIS rosdeps
  rosdep install \
    --os=ubuntu:${CODENAME} \
    --from-paths "${STACK_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    --default-yes \
    --skip-keys="${ROSDEP_IGNORE_KEYS}" \
; fi

# Bootstrap the Arduino toolchain
"${STACK_DIRECTORY}/oasis_avr/bootstrap.sh"

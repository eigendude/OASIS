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

# Import Python paths
source "${SCRIPT_DIR}/env_python.sh"

#
# Load ROS 2 environment
#

# Load ROS 2 Desktop
set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

# Install development tools and ROS tools
sudo apt install -y \
  python3-rosdep
python3 -m pip install --upgrade \
  colcon-common-extensions \
  vcstool

#
# Download OASIS dependency source code
#

vcs import "${OASIS_SOURCE_DIRECTORY}" < "${PACKAGE_DIRECTORY}/config/depends.repos"

# Patch dependency sources
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0001-Revert-Cleanup-the-cmake-code-to-be-more-modern-96.patch" \
  || :

#
# Install rosdeps packages (except on macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
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
    --as-root=pip:false \
    -y

  # Patch dependency sources
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${OASIS_SOURCE_DIRECTORY}/ros-perception/bgslibrary" \
    < "${CONFIG_DIRECTORY}/bgslibrary/0001-CMake-Add-missing-header-install-target.patch" \
    || :
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${OASIS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
    < "${CONFIG_DIRECTORY}/image_transport_plugins/0001-Revert-Add-tiff-compression-support.-75.patch" \
    || :

  # Add ccache support
  dpkg -s ccache >/dev/null || sudo apt install -y ccache

  # TODO: image_transport needs libtinyxml2-dev indirectly
  dpkg -s libtinyxml2-dev >/dev/null || sudo apt install -y libtinyxml2-dev

  # Observed on Ubuntu 20.04 that yaml-cpp wasn't installed
  # Needed for package camera_calibration_parsers
  dpkg -s libyaml-cpp-dev >/dev/null || sudo apt install -y libyaml-cpp-dev

  # Only enable OpenNI on x86_64
  if [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    # Needed for OpenNI
    for package in \
        default-jdk \
        libudev-dev \
        libusb-1.0-0-dev \
    ; do
      dpkg -s ${package} >/dev/null || sudo apt install -y ${package}
    done
  else
    echo "Disabling OpenNI on ${PLATFORM_ARCH}"
    touch "${OASIS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
  fi

  # Needed for libfreenect2
  for package in \
      libturbojpeg0-dev \
      libusb-1.0-0-dev \
      ocl-icd-opencl-dev \
  ; do
    dpkg -s ${package} >/dev/null || sudo apt install -y ${package}
  done
else
  echo "Disabling perception dependencies on macOS"
  touch "${OASIS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
  touch "${OASIS_SOURCE_DIRECTORY}/ros-perception/oasis_kinect2/COLCON_IGNORE"
  touch "${OASIS_SOURCE_DIRECTORY}/ros-perception/ros2-v4l2-camera/COLCON_IGNORE"
fi

#
# Install brew packages (macOS)
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update

  brew install \
    boost-python3 \
    libusb
fi

#
# Install required Python packages

python3 -m pip install --upgrade \
  numpy

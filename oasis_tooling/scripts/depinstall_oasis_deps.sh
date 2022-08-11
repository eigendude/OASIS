#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2022 Garrett Brown
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

# Import OASIS depends paths and config
source "${SCRIPT_DIR}/env_oasis_deps.sh"

#
# Load ROS 2 environment
#

# Load ROS 2 Desktop
set +o nounset
source "${ROS2_DESKTOP_DIRECTORY}/install/setup.bash"
set -o nounset

#
# Setup ROS 2 sources
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Add the ROS 2 repository
  ARCH="$(dpkg --print-architecture)"
  CODENAME="$(source "/etc/os-release" && echo "${UBUNTU_CODENAME}")"
  KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
  PKG_URL="http://packages.ros.org/ros2/ubuntu"
  SIGNED_BY="/usr/share/keyrings/ros-archive-keyring.gpg"
  SOURCES_LIST="/etc/apt/sources.list.d/ros2.list"

  if [ ! -f "${SIGNED_BY}" ] || [ ! -f ${SOURCES_LIST} ]; then
    # Install required dependencies
    sudo apt install -y \
      curl \
      gnupg2 \
      lsb-release

    # Authorize the ROS 2 GPG key with apt
    sudo curl -sSL "${KEY_URL}" -o "${SIGNED_BY}"

    # Add the ROS 2 repository to our sources list
    echo "deb [arch=${ARCH} signed-by=${SIGNED_BY}] ${PKG_URL} ${CODENAME} main" | \
      sudo tee "${SOURCES_LIST}"

    sudo apt update
  fi
else
  CODENAME=
fi

#
# Install build dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install development tools and ROS tools
  sudo apt install -y \
    build-essential \
    git \
    python3-pip \
    wget \

  # python3-rosdep is no longer an Ubuntu package, so install via pip
  sudo python3 -m pip install --upgrade rosdep

  # TODO: image_transport needs libtinyxml2-dev indirectly
  sudo apt install -y libtinyxml2-dev

  # Observed on Ubuntu 20.04 that yaml-cpp wasn't installed
  # Needed for package camera_calibration_parsers
  sudo apt install -y libyaml-cpp-dev

  # Observed on Ubuntu 22.04 that console_bridge wasn't located by CMake
  sudo apt install -y libconsole-bridge-dev

  # Only enable OpenNI on x86_64
  if [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    # Needed for OpenNI
    sudo apt install -y \
      default-jdk \
      libudev-dev \
      libusb-1.0-0-dev
  fi

  # Needed for libfreenect2
  sudo apt install -y \
    libturbojpeg0-dev \
    libusb-1.0-0-dev \
    ocl-icd-opencl-dev \

  # Add ccache support
  sudo apt install -y ccache
fi

#
# Install build dependencies (macOS)
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update

  brew install \
    boost-python3 \
    libusb
fi

#
# Install Python packages
#

# Upgraded setuptools may be required for other tools
python3 -m pip install --user --upgrade \
  pip \
  setuptools \

# Install development tools and ROS tools
python3 -m pip install --user --upgrade \
  colcon-common-extensions \
  rosdep \
  vcstool \

# numpy is also required
python3 -m pip install --user --upgrade \
  numpy \

# Sometimes lark is missing
python3 -m pip install --user --upgrade \
  importlib-resources \
  lark-parser \

#
# Directory setup
#

# Ensure directories exist
mkdir -p "${OASIS_DEPENDS_SOURCE_DIRECTORY}"
mkdir -p "${OASIS_DEPENDS_LIB_DIRECTORY}"

#
# Download OASIS dependency source code
#

vcs import "${OASIS_DEPENDS_SOURCE_DIRECTORY}" < "${PACKAGE_DIRECTORY}/config/depends.repos"

# Patch dependency sources
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/bgslibrary" \
  < "${CONFIG_DIRECTORY}/bgslibrary/0001-CMake-Add-missing-header-install-target.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0001-Revert-Add-tiff-compression-support.-75.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0001-Revert-Cleanup-the-cmake-code-to-be-more-modern-96.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0001-Fix-publisher-advertiseImpl-for-compressed-image-tra.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0002-Fix-compressed_depth_publisher.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0003-Fix-compressed_subscriber.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_transport_plugins" \
  < "${CONFIG_DIRECTORY}/image_transport_plugins/0004-Fix-theora_subscriber.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/vision_opencv" \
  < "${CONFIG_DIRECTORY}/vision_opencv/0001-Remove-boost-python-dependency.patch" \
  || :

# Disable OpenNI on everything but x86_64
if [[ ${PLATFORM_ARCH} != x86_64 ]]; then
  echo "Disabling OpenNI on ${PLATFORM_ARCH}"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
fi

# Disable bgslibrary on 18.04 due to older OpenCV version
if [ "${CODENAME}" = "bionic" ]; then
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/bgslibrary/COLCON_IGNORE"
fi

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
    --from-paths "${OASIS_DEPENDS_SOURCE_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    -y
else
  echo "Disabling perception dependencies on macOS"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/oasis_kinect2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/ros2-v4l2-camera/COLCON_IGNORE"
fi

#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
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
    ccache \
    git \
    python3-pip \

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

  # Install libcec dependencies
  sudo apt install -y \
    cmake \
    libudev-dev \
    libxrandr-dev \
    swig

  # Needed for libcec on the Raspberry Pi
  if [ "$(cat /proc/cpuinfo | grep "^Model" | awk '{print $3}')" = "Raspberry" ]; then
    sudo apt install -y libraspberrypi-dev
  fi

  # Needed by cv_bridge, dev dependency of tracetools package in ros2_tracing stack
  sudo apt install -y liblttng-ust-dev
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

#
# Patch dependency sources
#

# Disable bgslibrary on 18.04 due to older OpenCV version
if [ "${CODENAME}" = "bionic" ]; then
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/bgslibrary/COLCON_IGNORE"
fi

# libcec
cp -v \
  "${CONFIG_DIRECTORY}/libcec/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libcec"
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libcec" \
  < "${CONFIG_DIRECTORY}/libcec/0001-Ament-Add-packaging-for-ROS-2-compatibility.patch" \
  || :

# libfreenect2
cp -v \
  "${CONFIG_DIRECTORY}/libfreenect2/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2"
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0001-Add-ament-packaging.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0002-Change-CMake-option-defaults.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0003-Enable-PIC.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0004-Force-disable-components.patch" \
  || :

# OpenNI
cp -v \
  "${CONFIG_DIRECTORY}/OpenNI2/CMakeLists.txt" \
  "${CONFIG_DIRECTORY}/OpenNI2/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2"

# Disable OpenNI on everything but x86_64
if [[ ${PLATFORM_ARCH} != x86_64 ]]; then
  echo "Disabling OpenNI on ${PLATFORM_ARCH}"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
fi

# ORB-SLAM3
cp -v \
  "${CONFIG_DIRECTORY}/orb-slam3/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3"
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3" \
  < "${CONFIG_DIRECTORY}/orb-slam3/0001-Update-CMAKEList.txt-to-use-C-14.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3" \
  < "${CONFIG_DIRECTORY}/orb-slam3/0002-Add-missing-DBoW2-directory-to-CMakeLists.txt.patch" \
  || :
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3" \
  < "${CONFIG_DIRECTORY}/orb-slam3/0003-Add-install-step-to-CMakeLists.txt.patch" \
  || :

# Disable ORB-SLAM3 on Ubuntu 18.04 and Ubuntu 20.04
if [ "${CODENAME}" = "bionic" ] || [ "${CODENAME}" = "focal" ]; then
  echo "Disabling ORB-SLAM3 on ${CODENAME}"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3/COLCON_IGNORE"
fi

# Disable ORB_SLAM3 on systems with < 4GiB memory
PHYSICAL_MEMORY_KB=$(grep MemTotal /proc/meminfo | awk '{print $2}')
PHYSICAL_MEMORY_GB=$(echo "scale=4; ${PHYSICAL_MEMORY_KB}/1024^2" | bc)
if (( $(echo "${PHYSICAL_MEMORY_GB} < 4" | bc -l) )); then
  echo "Disabling ORB-SLAM3 with ${PHYSICAL_MEMORY_GB} GiB of RAM"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/orb-slam3/COLCON_IGNORE"
fi

# p8-platform
cp -v \
  "${CONFIG_DIRECTORY}/p8-platform/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/p8-platform"
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/p8-platform" \
  < "${CONFIG_DIRECTORY}/p8-platform/0001-CMake-Default-to-C-20-to-fix-building-with-newer-cla.patch" \
  || :

# Pangolin
patch \
  -p1 \
  --forward \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/pangolin" \
  < "${CONFIG_DIRECTORY}/pangolin/0001-Remove-Eigen3-Eigen-target-from-linked-libraries.patch" \
  || :

# ros2_v4l2_camera
if [ "${ROS2_DISTRO}" = "iron" ]; then
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/ros2_v4l2_camera" \
    < "${CONFIG_DIRECTORY}/ros2_v4l2_camera/0001-Fix-include-path.patch" \
    || :
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
  echo "Downloading dependency rosdeps..."

  # Install dependency rosdeps
  rosdep install \
    --from-paths "${OASIS_DEPENDS_SOURCE_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    --default-yes
else
  echo "Disabling perception dependencies on macOS"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/oasis_kinect2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/ros2-v4l2-camera/COLCON_IGNORE"
fi

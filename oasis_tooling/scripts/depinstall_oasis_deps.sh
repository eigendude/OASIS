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

# Import OASIS depends paths and config
source "${SCRIPT_DIR}/env_oasis_deps.sh"

# rosdep keys to ignore
ROSDEP_IGNORE_KEYS=" \
  ament_black \
  ament_cmake_black \
  image_view \
  launch_testing \
  launch_testing_ament_cmake \
  launch_testing_ros \
  libcamera \
  libopencv-dev \
  libopencv-imgproc-dev \
  python_cmake_module \
  python3-opencv \
  ros_testing \
"

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
  "${SCRIPT_DIR}/setup_ros2_desktop.sh"
fi

#
# Install build dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Packages to install via apt
  APT_PACKAGES=(
    # Development tools and ROS tools
    bc
    build-essential
    ccache
    cmake
    git
    ros-dev-tools

    # Needed for ROS message generation
    python3-lark

    # Needed by cv_bridge, dev dependency of tracetools package in ros2_tracing stack
    liblttng-ust-dev

    # Needed by image_pipeline
    liborocos-kdl-dev

    # Needed by image_transport and plugins
    libconsole-bridge-dev
    libspdlog-dev
    libtinyxml2-dev

    # Needed for image_view
    libavif-dev

    # Needed for v4l2_camera
    libyaml-cpp-dev

    # Needed for custom OpenCV build
    libopenblas-dev
    libopenexr-dev
    libprotobuf-dev

    # Needed for libfreenect2
    libturbojpeg0-dev
    libusb-1.0-0-dev
    ocl-icd-opencl-dev

    # libcec dependencies
    libudev-dev
    libxrandr-dev
    swig

    # Needed for libcamera according to https://github.com/christian-nils/libcamera_cmake
    cmake
    git
    libboost-dev
    libevent-dev
    libglib2.0-dev
    libgnutls28-dev
    libgstreamer-plugins-base1.0-dev
    libqt5core5a
    libqt5gui5
    libqt5widgets5
    libtiff5-dev
    meson
    ninja-build
    openssl
    pybind11-dev
    python3
    python3-jinja2
    python3-pip
    python3-ply
    python3-yaml
    qtbase5-dev
    v4l-utils
  )

  # Needed for OpenNI (only enabled on x86_64)
  if [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    APT_PACKAGES+=(
      default-jdk
      libudev-dev
      libusb-1.0-0-dev
    )
  fi

  # Needed for libcec on ARM
  if [[ "$(uname -m)" == "aarch64" ]] ||
     [[ "$(uname -m)" == "armv7l" ]] ||
     [[ "$(uname -m)" == "armv6l" ]]; then
    APT_PACKAGES+=(
      libraspberrypi-dev
    )
  fi

  # Needed for OpenCL support for oasis_kinect2
  APT_PACKAGES+=(
    clinfo
    ocl-icd-opencl-dev
    opencl-clhpp-headers
  )
  if [[ ${PLATFORM_ARCH} == i*86 ]] || [[ ${PLATFORM_ARCH} == x86_64 ]]; then
    # Intel OpenCL implementation
    APT_PACKAGES+=(
      intel-opencl-icd
    )
  else
    # Portable OpenCL implementation
    APT_PACKAGES+=(
      pocl-opencl-icd
    )
  fi

  sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"
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
# Directory setup
#

# Ensure a clean slate for source code without deleting directories
mkdir -p "${OASIS_DEPENDS_SOURCE_DIRECTORY}"
if [[ -d "${OASIS_DEPENDS_SOURCE_DIRECTORY}" ]]; then
  find "${OASIS_DEPENDS_SOURCE_DIRECTORY}" -name .git -type d | while read -r git_dir; do
    repo_dir="$(dirname "${git_dir}")"
    git -C "${repo_dir}" reset --hard HEAD
    git -C "${repo_dir}" clean -fdx
  done
fi

#
# Download OASIS dependency source code
#

vcs import --force "${OASIS_DEPENDS_SOURCE_DIRECTORY}" < "${PACKAGE_DIRECTORY}/config/depends.repos"

#
# Patch dependency sources
#

# bgslibrary
echo "Patching bgslibrary..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/bgslibrary" \
  < "${CONFIG_DIRECTORY}/bgslibrary/0001-Disable-imshow-calls.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/bgslibrary" \
  < "${CONFIG_DIRECTORY}/bgslibrary/0002-CMake-Fix-locating-libs-via-_ROOT-variables.patch"

# camera_ros
echo "Patching camera_ros..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/camera_ros" \
  < "${CONFIG_DIRECTORY}/camera_ros/0001-Use-modern-image_transport-API.patch"

# Disable image_view on systems with <= 4GiB memory
if (( $(echo "${PHYSICAL_MEMORY_GB} <= 4" | bc -l) )); then
  echo "Disabling image_view with ${PHYSICAL_MEMORY_GB} GiB of RAM"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/image_pipeline/image_view/COLCON_IGNORE"
fi

# libcec
echo "Patching libcec..."
cp -v \
  "${CONFIG_DIRECTORY}/libcec/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libcec"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libcec" \
  < "${CONFIG_DIRECTORY}/libcec/0001-Ament-Add-packaging-for-ROS-2-compatibility.patch"

# libfreenect2
echo "Patching libfreenect2..."
cp -v \
  "${CONFIG_DIRECTORY}/libfreenect2/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0001-Add-ament-packaging.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0002-Change-CMake-option-defaults.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0003-Enable-PIC.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/libfreenect2" \
  < "${CONFIG_DIRECTORY}/libfreenect2/0004-Force-disable-components.patch"

# OpenNI
echo "Patching OpenNI2..."
cp -v \
  "${CONFIG_DIRECTORY}/OpenNI2/CMakeLists.txt" \
  "${CONFIG_DIRECTORY}/OpenNI2/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2"

# Disable OpenNI on everything but x86_64
if [[ ${PLATFORM_ARCH} != x86_64 ]]; then
  echo "Disabling OpenNI on ${PLATFORM_ARCH}"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
fi

# ORB_SLAM_OASIS
echo "Patching ORB_SLAM_OASIS..."
cp -v \
  "${CONFIG_DIRECTORY}/ORB_SLAM_OASIS/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/ORB_SLAM_OASIS"

# Disable ORB_SLAM_OASIS on systems with <= 4GiB memory
if (( $(echo "${PHYSICAL_MEMORY_GB} <= 4" | bc -l) )); then
  echo "Disabling ORB_SLAM_OASIS with ${PHYSICAL_MEMORY_GB} GiB of RAM"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/ORB_SLAM_OASIS/COLCON_IGNORE"
fi

# p8-platform
echo "Patching p8-platform..."
cp -v \
  "${CONFIG_DIRECTORY}/p8-platform/package.xml" \
  "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/p8-platform"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/p8-platform" \
  < "${CONFIG_DIRECTORY}/p8-platform/0001-CMake-Default-to-C-20-to-fix-building-with-newer-cla.patch"

# Pangolin
echo "Patching Pangolin..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/pangolin" \
  < "${CONFIG_DIRECTORY}/pangolin/0001-Remove-Eigen3-Eigen-target-from-linked-libraries.patch"

# Disable pangolin on systems with <= 4GiB memory
if (( $(echo "${PHYSICAL_MEMORY_GB} <= 4" | bc -l) )); then
  echo "Disabling pangolin with ${PHYSICAL_MEMORY_GB} GiB of RAM"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/pangolin/COLCON_IGNORE"
fi

# ros2_v4l2_camera
echo "Patching ros2_v4l2_camera..."
if [ "${ROS2_DISTRO}" = "iron" ]; then
  patch \
    -p1 \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/ros2_v4l2_camera" \
    < "${CONFIG_DIRECTORY}/ros2_v4l2_camera/0001-Fix-include-path.patch"
fi

# vision_opencv
echo "Patching vision_opencv..."
touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/vision_opencv/opencv_tests/COLCON_IGNORE"

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
    --os=ubuntu:${CODENAME} \
    --from-paths "${OASIS_DEPENDS_SOURCE_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    --default-yes \
    --skip-keys="${ROSDEP_IGNORE_KEYS}"
else
  echo "Disabling perception dependencies on macOS"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/depends/OpenNI2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/oasis_kinect2/COLCON_IGNORE"
  touch "${OASIS_DEPENDS_SOURCE_DIRECTORY}/ros-perception/ros2-v4l2-camera/COLCON_IGNORE"
fi

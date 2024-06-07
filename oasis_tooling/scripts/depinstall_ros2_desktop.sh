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
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

# rosdep keys to ignore
if [ "${ROS2_DISTRO}" = "iron" ] || [ "${ROS2_DISTRO}" = "jazzy" ]; then
  ROSDEP_IGNORE_KEYS=" \
    fastcdr \
    rti-connext-dds-6.0.1 \
    urdfdom_headers \
  "
else
  ROSDEP_IGNORE_KEYS=
fi

if [[ "${OSTYPE}" != "darwin"* ]]; then
  ARCH="$(dpkg --print-architecture)"
  CODENAME="$(source "/etc/os-release" && echo "${UBUNTU_CODENAME}")"
else
  ARCH=
  CODENAME=
fi

#
# Setup ROS 2 sources
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Add the ROS 2 repository
  KEY_URL="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
  PKG_URL="http://packages.ros.org/ros2/ubuntu"
  SIGNED_BY="/usr/share/keyrings/ros-archive-keyring.gpg"
  SOURCES_LIST="/etc/apt/sources.list.d/ros2.list"

  if [ ! -f "${SIGNED_BY}" ] || [ ! -f ${SOURCES_LIST} ]; then
  # Install required dependencies
    sudo apt install -y \
      curl \
      gnupg2 \
      lsb-release \

    # Authorize the ROS 2 GPG key with apt
    sudo curl -sSL "${KEY_URL}" -o "${SIGNED_BY}"

    # Add the ROS 2 repository to our sources list
    echo "deb [arch=${ARCH} signed-by=${SIGNED_BY}] ${PKG_URL} ${CODENAME} main" | \
      sudo tee "${SOURCES_LIST}"

    sudo apt update
  fi
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
    gfortran \
    libbullet-dev \
    python3-pip \
    wget \

  # Install Fast-RTPS dependencies
  sudo apt install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \

  # Install Cyclone DDS dependencies
  sudo apt install -y --no-install-recommends \
    libcunit1-dev \

  # Install development tools
  sudo apt install -y --no-install-recommends \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools
fi

#
# Install build dependencies (macOS)
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update

  # Note: qt@6 causes problems with rviz_rendering, make sure qt@6 isn't installed
  brew install \
    asio \
    assimp \
    bison \
    bullet \
    ccache \
    cmake \
    console_bridge \
    cppcheck \
    cunit \
    eigen \
    freetype \
    graphviz \
    log4cxx \
    opencv \
    openssl \
    pcre \
    poco \
    pyqt5 \
    python \
    qt@5 \
    sip \
    spdlog \
    tinyxml \
    tinyxml2 \
    wget \

  python3 -m pip install --upgrade \
    pip \
    setuptools \

  python3 -m pip install --upgrade \
    argcomplete \
    catkin_pkg \
    colcon-common-extensions \
    coverage \
    cryptography \
    empy \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    ifcfg \
    importlib-metadata \
    lark-parser \
    lxml \
    matplotlib \
    mock \
    mypy==0.761 \
    netifaces \
    nose \
    pep8 \
    psutil \
    pydocstyle \
    pydot \
    pyparsing \
    pytest-mock \
    rosdep \
    rosdistro \
    vcstool \

  GRAPHVIZ_VERSION=$(brew list --version | grep graphviz | cut -d " " -f 2)
  python3 -m pip install --upgrade \
    --global-option=build_ext \
    --global-option="-I/usr/local/Cellar/graphviz/${GRAPHVIZ_VERSION}/include" \
    --global-option="-L/usr/local/Cellar/graphviz/${GRAPHVIZ_VERSION}/lib" \
    pygraphviz
fi

#
# Directory setup
#

# Ensure directories exist
mkdir -p "${ROS2_SOURCE_DIRECTORY}"
mkdir -p "${ROS2_INSTALL_DIRECTORY}"

# After updating to Ubuntu 22.04, ament packages couldn't be found because
# they were installed to a different directory
if [ ! -L "${AMENT_INSTALL_DIRECTORY}" ]; then
  rm -rf "${AMENT_INSTALL_DIRECTORY}"
  ln -s "${ROS2_INSTALL_DIRECTORY}" "${AMENT_INSTALL_DIRECTORY}"
fi

#
# Download the ROS 2 source code
#

echo "Downloading ROS 2 source code..."
(
  cd "${ROS2_SOURCE_DIRECTORY}"

  # Get ROS 2 source defintions
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/${ROS2_DISTRO}/ros2.repos"

  # Import ROS 2 sources
  vcs import "${ROS2_SOURCE_DIRECTORY}" < ros2.repos
)

#
# Install rosdep packages
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install rosdep
  [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ] || sudo rosdep init

  # Update rosdep
  rosdep update

  # Install rosdep packages
  echo "Installing rosdep packages..."
  rosdep install \
    --os=ubuntu:${CODENAME} \
    --from-paths "${ROS2_SOURCE_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    --default-yes \
    --skip-keys "${ROSDEP_IGNORE_KEYS}"
fi

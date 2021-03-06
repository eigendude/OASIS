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
# Install dependencies (everything but macOS)
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # Install required dependencies
  sudo apt install -y curl gnupg2 lsb-release make python3-rosdep

  # Add the ROS 2 apt repository
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

  # Add the ROS 2 repository to our sources list
  # NOTE: hersute packages not currently available
  echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list

  sudo apt update

  # Install development tools and ROS tools
  sudo apt install -y \
    build-essential \
    git \
    gfortran \
    libbullet-dev \
    wget
  # Upgraded setuptools may be required for other tools
  python3 -m pip install --upgrade \
    pip \
    setuptools
  python3 -m pip install --upgrade \
    colcon-common-extensions \
    flake8 \
    pytest-cov \
    vcstool

  # Install some pip packages needed for testing
  python3 -m pip install --upgrade \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest \
    pytest-repeat \
    pytest-rerunfailures

  # Install Fast-RTPS dependencies
  sudo apt install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev

  # Install Cyclone DDS dependencies
  sudo apt install -y --no-install-recommends \
    libcunit1-dev

  # Sometimes lark is missing
  python3 -m pip install --upgrade \
    importlib-resources \
    lark-parser

  # This is needed by rosidl_generator_py
  python3 -m pip install --upgrade \
    numpy

  # ROS 2 runtime dependencies
  python3 -m pip install --upgrade \
    netifaces
fi

#
# Install dependencies (macOS)
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
    wget

  python3 -m pip install --upgrade \
    setuptools
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
    vcstool

  GRAPHVIZ_VERSION=$(brew list --version | grep graphviz | cut -d " " -f 2)
  python3 -m pip install --upgrade \
    --global-option=build_ext \
    --global-option="-I/usr/local/Cellar/graphviz/${GRAPHVIZ_VERSION}/include" \
    --global-option="-L/usr/local/Cellar/graphviz/${GRAPHVIZ_VERSION}/lib" \
    pygraphviz
fi

#
# Download the ROS 2 source code
#

echo "Downloading ROS 2 source code..."
(
  cd "${ROS2_SOURCE_DIRECTORY}"

  # Get ROS 2 sources
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/${ROS2_DISTRO}/ros2.repos"

  # Patch ROS 2 sources
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    < "${CONFIG_DIRECTORY}/ros2_desktop/0001-Change-rcpputils-to-master-branch.patch"

  # Import ROS 2 sources
  vcs import "${ROS2_SOURCE_DIRECTORY}" < ros2.repos

  # Patch ROS 2 packages
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS2_SOURCE_DIRECTORY}/eclipse-iceoryx/iceoryx" \
    < "${CONFIG_DIRECTORY}/iceoryx/0001-Fix-static_asserts-causing-build-to-fail.patch" \
    || :
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS2_SOURCE_DIRECTORY}/ros2/rviz" \
    < "${CONFIG_DIRECTORY}/rviz/0001-Update-to-C-17.patch" \
    || :
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
    --from-paths "${ROS2_SOURCE_DIRECTORY}" \
    --ignore-src \
    --rosdistro ${ROS2_DISTRO} \
    --as-root=pip:false \
    -y \
    --skip-keys "console_bridge fastcdr fastrtps python3-ifcfg rti-connext-dds-5.3.1 urdfdom_headers"

  # Add ccache support
  dpkg -s ccache >/dev/null || sudo apt install -y ccache
fi

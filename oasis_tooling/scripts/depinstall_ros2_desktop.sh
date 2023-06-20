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

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

# rosdep keys to ignore
if [ "${ROS2_DISTRO}" = "humble" ]; then
  ROSDEP_IGNORE_KEYS=" \
    fastcdr \
    ignition-cmake2 \
    ignition-math6 \
    python-catkin-pkg \
    python3-pykdl \
    rti-connext-dds-6.0.1 \
    urdfdom_headers \
  "
elif [ "${ROS2_DISTRO}" = "iron" ]; then
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

  # On Ubuntu 18.04, GCC 7 cannot build rclcpp
  if [ "${CODENAME}" = "bionic" ]; then
    sudo apt install -y g++-8
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 700 --slave /usr/bin/g++ g++ /usr/bin/g++-7
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8
  fi

  # python3-rosdep is no longer an Ubuntu package, so install via pip
  sudo python3 -m pip install --upgrade rosdep

  # Install Fast-RTPS dependencies
  sudo apt install -y --no-install-recommends \
    libasio-dev \
    libtinyxml2-dev \

  # Install Cyclone DDS dependencies
  sudo apt install -y --no-install-recommends \
    libcunit1-dev \

  # Upgraded setuptools may be required for other tools
  python3 -m pip install --user --upgrade \
    pip \
    setuptools \

  # Install development tools and ROS tools
  python3 -m pip install --user --upgrade \
    colcon-common-extensions \
    rosdep \
    vcstool \

  # Install some pip packages needed for testing
  python3 -m pip install --user --upgrade \
    argcomplete \
    black \
    click \
    distro \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    mypy \
    mypy-extensions \
    pytest \
    pytest-cov \
    pytest-repeat \
    pytest-rerunfailures \
    tox \

  # Sometimes lark is missing
  python3 -m pip install --user --upgrade \
    importlib-resources \
    lark-parser \

  # This is needed by rosidl_generator_py
  python3 -m pip install --user --upgrade \
    numpy \

  # ROS 2 runtime dependencies
  python3 -m pip install --user --upgrade \
    netifaces

  if [ "${ROS2_DISTRO}" = "humble" ]; then
    # Needed by python_orocos_kdl_vendor
    python3 -m pip install --user --upgrade \
      pybind11 \
      pybind11-global
  fi
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
mkdir -p "${ROS2_PYTHON_PKG_DIRECTORY}"

# After updating to Ubuntu 22.04, ament packages couldn't be found becaues
# they were installed to a different directory
if [ ! -L "${AMENT_INSTALL_DIRECTORY}" ]; then
  rm -rf "${AMENT_INSTALL_DIRECTORY}"
  ln -s "${ROS2_INSTALL_DIRECTORY}" "${AMENT_INSTALL_DIRECTORY}"
fi

if [ ! -L "${AMENT_PYTHON_PKG_DIRECTORY}" ]; then
  rm -rf "${AMENT_PYTHON_PKG_DIRECTORY}"
  ln -s "${ROS2_PYTHON_PKG_DIRECTORY}" "${AMENT_PYTHON_PKG_DIRECTORY}"
fi

#
# Download the ROS 2 source code
#

echo "Downloading ROS 2 source code..."
(
  cd "${ROS2_SOURCE_DIRECTORY}"

  # Get ROS 2 source defintions
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/${ROS2_DISTRO}/ros2.repos"

  # Patch ROS 2 source definitions
  if [ "${ROS2_DISTRO}" = "humble" ]; then
    patch \
      -p1 \
      --forward \
      --reject-file="/dev/null" \
      --no-backup-if-mismatch \
      < "${CONFIG_DIRECTORY}/ros2_desktop/0001-humble-Update-image_common-to-rolling-branch.patch"
  fi

  # Import ROS 2 sources
  vcs import "${ROS2_SOURCE_DIRECTORY}" < ros2.repos

  # Patch ROS 2 packages
  if [ "${ROS2_DISTRO}" = "humble" ]; then
    patch \
      -p1 \
      --forward \
      --reject-file="/dev/null" \
      --no-backup-if-mismatch \
      --directory="${ROS2_SOURCE_DIRECTORY}/ros2/rviz" \
      < "${CONFIG_DIRECTORY}/rviz/0001-Update-to-C-17.patch" \
      || :

    patch \
      -p1 \
      --forward \
      --reject-file="/dev/null" \
      --no-backup-if-mismatch \
      --directory="${ROS2_SOURCE_DIRECTORY}/eclipse-iceoryx/iceoryx" \
      < "${CONFIG_DIRECTORY}/iceoryx/0001-humble-Fix-static_asserts-causing-build-to-fail.patch" \
      || :

    if [ "${CODENAME}" = "bionic" ]; then
      # Bionic version of yaml-cpp (0.5) is too old
      patch \
        -p1 \
        --forward \
        --reject-file="/dev/null" \
        --no-backup-if-mismatch \
        --directory="${ROS2_SOURCE_DIRECTORY}/ros2/yaml_cpp_vendor" \
        < "${CONFIG_DIRECTORY}/yaml_cpp_vendor/0001-Default-to-building-yaml-cpp-from-source.patch" \
        || :

      # On Ubuntu 18.04, intra_process_demo fails to link against libopencv_imgproc.so.3.2
      touch "${ROS2_SOURCE_DIRECTORY}/ros2/demos/intra_process_demo/COLCON_IGNORE"
    fi

    # TODO: Disabled to increase build speed
    echo "Disabling rviz"
    touch "${ROS2_SOURCE_DIRECTORY}/ros2/rviz/COLCON_IGNORE"
  fi
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
    --skip-keys "${ROSDEP_IGNORE_KEYS}" \

  # Package provided by Ubuntu 18.04 is too old
  if [ "${ROS2_DISTRO}" = "humble" ] && [ "${CODENAME}" = "bionic" ]; then
    sudo apt remove -y libyaml-cpp0.5v5
  fi
fi

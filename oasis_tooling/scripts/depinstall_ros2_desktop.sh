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
  # Testing ignorables
  ROSDEP_IGNORE_KEYS+=" \
    launch_testing \
    launch_testing_ament_cmake \
    launch_testing_ros \
    osrf_testing_tools_cpp \
    performance_test_fixture \
    ros2cli_test_interfaces \
    ros2lifecycle_test_fixtures \
    ros_testing \
    rosbag2_test_common \
    rosbag2_test_msgdefs \
    rosbag2_tests \
    rviz_rendering_tests \
    rviz_visual_testing_framework \
    test_msgs \
  "
else
  ROSDEP_IGNORE_KEYS=
fi

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
  # Install general development tools
  sudo apt install -y --no-install-recommends \
    build-essential \
    ccache \
    cmake \
    git \
    wget \

  # Install ROS development tools
  sudo apt install -y --no-install-recommends \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-lark \
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

  # Install build utilities
  brew install \
    ccache \
    wget \

  # Install ROS dependencies
  # Note: qt@6 causes problems with rviz_rendering, make sure qt@6 isn't installed
  brew install \
    asio \
    assimp \
    bison \
    bullet \
    cmake \
    console_bridge \
    cppcheck \
    cunit \
    eigen \
    freetype \
    graphviz \
    opencv \
    openssl \
    orocos-kdl \
    pcre \
    poco \
    pyqt5 \
    python \
    qt@5 \
    sip \
    spdlog \
    tinyxml \
    tinyxml2 \

  # Update Python utilities
  python3 -m pip install --upgrade \
    pip \
    setuptools \

  # Install ROS Python dependencies
  python3 -m pip install --upgrade \
    argcomplete \
    catkin_pkg \
    colcon-common-extensions \
    coverage \
    cryptography \
    empy \
    flake8 \
    flake8-blind-except==0.1.1 \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    importlib-metadata \
    jsonschema \
    lark==1.1.1 \
    lxml \
    matplotlib \
    mock \
    mypy==0.931 \
    netifaces \
    nose \
    pep8 \
    psutil \
    pydocstyle \
    pydot \
    pyparsing==2.4.7 \
    pytest-mock \
    rosdep \
    rosdistro \
    setuptools==59.6.0 \
    vcstool \

  GRAPHVIZ_VERSION=$(brew list --version | grep graphviz | cut -d " " -f 2)
  python3 -m pip install --upgrade \
    --global-option=build_ext \
    --global-option="-I$(brew --prefix)/Cellar/graphviz/${GRAPHVIZ_VERSION}/include" \
    --global-option="-L$(brew --prefix)/Cellar/graphviz/${GRAPHVIZ_VERSION}/lib" \
    pygraphviz
fi

#
# Directory setup
#

# Ensure directories exist
mkdir -p "${ROS2_SOURCE_DIRECTORY}"
mkdir -p "${ROS2_INSTALL_DIRECTORY}"

if [[ "${OSTYPE}" != "darwin"* ]]; then
  # After updating to Ubuntu 22.04, ament packages couldn't be found because
  # they were installed to a different directory
  if [ ! -L "${AMENT_INSTALL_DIRECTORY}" ]; then
    rm -rf "${AMENT_INSTALL_DIRECTORY}"
    ln -s "${ROS2_INSTALL_DIRECTORY}" "${AMENT_INSTALL_DIRECTORY}"
  fi
fi

#
# Download the ROS 2 source code
#

echo "Downloading ROS 2 source code..."
(
  cd "${ROS2_SOURCE_DIRECTORY}"

  # Get ROS 2 source defintions
  wget --timestamping "https://raw.githubusercontent.com/ros2/ros2/${ROS2_DISTRO}/ros2.repos"

  # Update image_common branch
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS2_SOURCE_DIRECTORY}" \
    < "${CONFIG_DIRECTORY}/ros2-desktop/0001-Change-image_common-to-rolling-branch.patch"

  # Import ROS 2 sources
  vcs import "${ROS2_SOURCE_DIRECTORY}" < ros2.repos
)

#
# Disable examples, demos, tutorials, benchmarks and tests
#

touch "${ROS2_SOURCE_DIRECTORY}/osrf/osrf_testing_tools_cpp/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros/ros_tutorials/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/demos/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/example_interfaces/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/examples/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/geometry2/examples_tf2_py/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/geometry2/test_tf2/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch/launch_pytest/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch/launch_testing/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch/launch_testing_ament_cmake/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch/test_launch_testing/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch_ros/launch_testing_ros/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/launch_ros/test_launch_ros/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/performance_test_fixture/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rcl_interfaces/test_msgs/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/realtime_support/rttest/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rmw_implementation/test_rmw_implementation/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2_tracing/test_ros2trace/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2_tracing/test_tracetools/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2_tracing/test_tracetools_launch/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2_tracing/tracetools_test/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2cli/ros2cli_test_interfaces/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros2cli/ros2lifecycle_test_fixtures/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/ros_testing/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosbag2/rosbag2_examples/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosbag2/rosbag2_performance/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosbag2/rosbag2_test_common/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosbag2/rosbag2_test_msgdefs/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosbag2/rosbag2_tests/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosidl/rosidl_generator_tests/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosidl/rosidl_typesupport_introspection_tests/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rosidl_typesupport/rosidl_typesupport_tests/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rviz/rviz_rendering_tests/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/rviz/rviz_visual_testing_framework/COLCON_IGNORE"
touch "${ROS2_SOURCE_DIRECTORY}/ros2/system_tests/COLCON_IGNORE"

# Disable rviz on systems with <= 4GiB memory
if (( $(echo "${PHYSICAL_MEMORY_GB} <= 4" | bc -l) )); then
  echo "Disabling rviz with ${PHYSICAL_MEMORY_GB} GiB of RAM"
  touch "${ROS2_SOURCE_DIRECTORY}/ros2/rviz/COLCON_IGNORE"
fi

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

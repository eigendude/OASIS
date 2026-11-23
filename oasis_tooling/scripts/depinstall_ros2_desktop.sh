#!/bin/bash
################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

if [[ "${OSTYPE}" == "darwin"* ]]; then
  brew update
  "${SCRIPT_DIR}/install_python.sh"
fi

# Import ROS 2 paths and config
source "${SCRIPT_DIR}/env_ros2_desktop.sh"

# rosdep keys to ignore
ROSDEP_IGNORE_KEYS=" \
  fastcdr \
  rti-connext-dds-7.7.0 \
  urdfdom_headers \
"

# Testing/development ignorables
ROSDEP_IGNORE_KEYS+=" \
  clang-format \
  launch_testing \
  launch_testing_ament_cmake \
  launch_testing_ros \
  osrf_testing_tools_cpp \
  performance_test_fixture \
  python3-matplotlib \
  python3-numpy \
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

# Ignore old releases of Rust and Cargo on older Ubuntu
if [[ "${OSTYPE}" != "darwin"* ]]; then
  source /etc/os-release
  if dpkg --compare-versions "${VERSION_ID}" lt "26.04"; then
    ROSDEP_IGNORE_KEYS+=" \
      cargo \
      rustc \
    "
  fi
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
  # Packages to install via apt
  APT_PACKAGES=(
    # General development tools
    build-essential
    ccache
    cmake
    git
    wget

    # ROS development tools
    python3-flake8-blind-except
    python3-flake8-class-newline
    python3-flake8-deprecated
    python3-lark
    python3-mypy
    python3-pip
    python3-pytest
    python3-pytest-cov
    python3-pytest-mock
    python3-pytest-repeat
    python3-pytest-rerunfailures
    python3-pytest-runner
    python3-pytest-timeout
    ros-dev-tools

    # Qt and SIP development files used by qt_gui_cpp
    pyqt6-dev
    python3-pyqt6
    python3-pyqtbuild
    qt6-base-dev
  )

  sudo apt install -y --no-install-recommends "${APT_PACKAGES[@]}"

  # NumPy is needed by rosidl_generator_py and cv_bridge.
  NUMPY_PYTHON_BIN="python3" \
    "${SCRIPT_DIR}/install_numpy.sh"
fi

#
# Install build dependencies (macOS)
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # Install build utilities
  brew install \
    ccache \
    rust \
    wget

  # Install ROS dependencies
  brew install \
    asio \
    assimp \
    bison \
    bullet \
    console_bridge \
    cppcheck \
    cunit \
    eigen \
    freetype \
    graphviz \
    libyaml \
    openssl \
    orocos-kdl \
    pcre \
    poco \
    pyqt \
    qtbase \
    spdlog \
    tinyxml2

  # Let workspace Python load the bindings built against Homebrew's Qt
  venv_site_packages="$(python3 -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  echo "$(brew --prefix pyqt)/lib/python${PYTHON_VERSION}/site-packages" \
    > "${venv_site_packages}/homebrew-pyqt.pth"

  # Graphviz headers and libraries are needed when pygraphviz builds from source
  CFLAGS="-I$(brew --prefix graphviz)/include ${CFLAGS:-}" \
    LDFLAGS="-L$(brew --prefix graphviz)/lib ${LDFLAGS:-}" \
    python3 -m pip install --upgrade \
      --requirement "${CONFIG_DIRECTORY}/ros2/requirements-macos.txt"

  # Verify that installed Python packages have compatible dependencies
  python3 -m pip check
fi

#
# Directory setup
#

# Ensure a clean slate for source code without deleting directories
mkdir -p "${ROS2_SOURCE_DIRECTORY}"
if [[ -d "${ROS2_SOURCE_DIRECTORY}" ]]; then
  find "${ROS2_SOURCE_DIRECTORY}" -name .git -type d | while read -r git_dir; do
    repo_dir="$(dirname "${git_dir}")"
    git -C "${repo_dir}" reset --hard HEAD
    git -C "${repo_dir}" clean -fdx
  done
fi
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

  # Import ROS 2 sources
  vcs import --force "${ROS2_SOURCE_DIRECTORY}" < ros2.repos
)

# Avoid collisions between nested message types with the same short name
if [[ "${ROS2_DISTRO}" == "rolling" ]]; then
  patch \
    -p1 \
    --forward \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS2_SOURCE_DIRECTORY}/ros2/rosidl_typesupport_fastrtps" \
    < "${CONFIG_DIRECTORY}/rosidl_typesupport_fastrtps/0001-Avoid-global-message-type-aliases.patch"
fi

# Use the Homebrew Qt bindings and native Ogre build settings on macOS
if [[ "${OSTYPE}" == "darwin"* ]]; then
  patch -p1 --directory="${ROS2_SOURCE_DIRECTORY}/ros-visualization/python_qt_binding" \
    < "${CONFIG_DIRECTORY}/python_qt_binding/0001-Select-PyQt-for-the-SIP-bindings-on-macOS.patch"
  patch \
    -p1 \
    --reject-file="/dev/null" \
    --no-backup-if-mismatch \
    --directory="${ROS2_SOURCE_DIRECTORY}/ros2/rviz" \
    < "${CONFIG_DIRECTORY}/rviz/0001-Fix-Ogre-SDK-and-architecture-on-macOS.patch"
fi

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

################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.5)
project(oasis_perception)

#
# Build configuration
#

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(bgslibrary REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#
# Includes
#

include_directories(
  ${BGSLIBRARY_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
)

#
# Executables
#

add_executable(background_subtractor
  src/image/BackgroundModeler.cpp
  src/image/MultiModeler.cpp
  src/ros/BackgroundModelerNode.cpp
  src/utils/NetworkUtils.cpp
  src/background_subtractor.cpp
)

ament_target_dependencies(background_subtractor
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

target_link_libraries(background_subtractor
  ${BGSLIBRARY_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${image_transport_LIBRARIES}
)

#
# Install
#

install(
  TARGETS
    background_subtractor
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# Testing
#

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

#
# Launch files
#

install(
  FILES
    launch/perception_launch.py
  DESTINATION
    share/${PROJECT_NAME}
)

#
# Systemd services
#

install(
  FILES
    config/systemd/oasis_perception.service
  DESTINATION
    share/${PROJECT_NAME}/systemd
)

#
# Declare an ament Package
#

ament_package()

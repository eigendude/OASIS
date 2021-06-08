################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of Oasis - https://github.com/eigendude/oasis
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.5)
project(oasis_drivers_cpp)

#
# Build configuration
#

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# TODO: -Wpedantic causes warnings due to libcec
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(CEC REQUIRED)
find_package(oasis_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(UDEV REQUIRED)

#
# C++ nodes
#

add_executable(cec_server
  src/cec/CecAdapter.cpp
  src/cec/CecServer.cpp
  src/cec/CecTranslator.cpp
  src/cec/CecUtils.cpp
  src/ros/CecServerNode.cpp
  src/udev/UdevScanner.cpp
  src/udev/UdevTranslator.cpp
  src/udev/UdevUtils.cpp
  src/utils/LogUtils.cpp
  src/utils/StringUtils.cpp
  src/cec_server.cpp
)

ament_target_dependencies(cec_server
  oasis_msgs
  rclcpp
  rcutils
  std_msgs
)

target_compile_definitions(cec_server PUBLIC
  ${CEC_DEFINITIONS}
  ${UDEV_DEFINITIONS}
)

target_include_directories(cec_server PUBLIC
  ${CEC_INCLUDE_DIRS}
  ${UDEV_INCLUDE_DIRS}
)

target_link_libraries(cec_server
  ${CEC_LIBRARIES}
  ${UDEV_LIBRARIES}
)

install(
  TARGETS
    cec_server
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
# Declare an ament Package
#

ament_package()

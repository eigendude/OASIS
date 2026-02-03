################################################################################
#
#  Copyright (C) 2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.10...4.0)
project(i2cdevlib)

#
# Dependencies
#

find_package(ament_cmake REQUIRED)

#
# Build configuration
#

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

#
# MPU 6050 library
#

add_library(
  i2cdevlib
  LinuxI2CDev/I2Cdev/I2Cdev.cpp
  LinuxI2CDev/MPU6050/MPU6050.cpp
)

target_include_directories(i2cdevlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LinuxI2CDev/I2Cdev>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/LinuxI2CDev/MPU6050>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(
  DIRECTORY
    LinuxI2CDev/I2Cdev/
    LinuxI2CDev/MPU6050/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
  TARGETS i2cdevlib
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

#
# Declare an ament Package
#

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()

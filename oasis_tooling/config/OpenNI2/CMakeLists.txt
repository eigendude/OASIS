################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.20)
project(openni2)

#
# Path configuration
#

set(OPENNI2_PLATFORM x64)
if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
  set(OPENNI2_PLATFORM Arm64)
endif()
set(OPENNI2_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Bin/${OPENNI2_PLATFORM}-Release")
set(OPENNI2_LIBRARY "${OPENNI2_BINARY_DIR}/libOpenNI2${CMAKE_SHARED_LIBRARY_SUFFIX}")

find_program(OPENNI2_MAKE make REQUIRED)
set(OPENNI2_BUILD_COMMAND "${OPENNI2_MAKE}" ALLOW_WARNINGS=1 GLUT_SUPPORTED=0)
if(APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
  # The upstream makefiles use MacPorts paths; provide Homebrew's USB prefix
  list(PREPEND OPENNI2_BUILD_COMMAND
    "${CMAKE_COMMAND}" -E env
    "CPATH=${LIBUSB_INCLUDEDIR}"
    "LIBRARY_PATH=${LIBUSB_LIBDIR}"
  )
endif()

#
# Targets
#

# Build the library and runtime drivers installed by this package
add_custom_command(
  OUTPUT
    ${OPENNI2_LIBRARY}
  COMMAND
    ${OPENNI2_BUILD_COMMAND}
      Source/Core
      Source/Drivers/DummyDevice
      Source/Drivers/PS1080
      Source/Drivers/PSLink
      Source/Drivers/OniFile
  WORKING_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(openni2 ALL
  DEPENDS
    ${OPENNI2_LIBRARY}
)

#
# Installation
#

install(
  DIRECTORY "${OPENNI2_BINARY_DIR}/OpenNI2"
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    Include
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  FILES
    ${OPENNI2_LIBRARY}
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# Declare an ament Package
#

find_package(ament_cmake REQUIRED)
ament_package()

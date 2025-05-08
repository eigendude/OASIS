################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.5)
project(openni2)

#
# Path configuration
#

set(OPENNI2_LIBRARY Bin/x64-Release/libOpenNI2.so)

#
# Targets
#

add_custom_command(
  OUTPUT
    ${OPENNI2_LIBRARY}
  COMMAND
    make ALLOW_WARNINGS=1 GLUT_SUPPORTED=0 -j$$(nproc)
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

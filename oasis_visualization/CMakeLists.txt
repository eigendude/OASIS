################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.10)
project(oasis_visualization)

#
# Dependencies
#

find_package(ament_cmake REQUIRED)

#
# Scripts
#

install(
  PROGRAMS
    scripts/dismiss-osd.sh
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# Configuration files
#

install(
  FILES
    config/systemd/dismiss-osd.service
    config/systemd/dismiss-osd.timer
    config/systemd/kodi_visualizer.service
  DESTINATION
    share/${PROJECT_NAME}/systemd
)

#
# Declare an ament Package
#

ament_package()

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
project(oasis_drivers_py)

#
# Dependencies
#

find_package(ament_cmake REQUIRED)

#
# Configuration files
#

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

#
# Declare an ament package
#

ament_package()

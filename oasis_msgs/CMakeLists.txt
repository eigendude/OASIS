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
project(oasis_msgs)

#
# Dependencies
#

find_package(rosidl_default_generators REQUIRED)

#
# Declare an ament Package
#

ament_package()

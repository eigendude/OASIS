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
project(oasis_msgs)

#
# Dependencies
#

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

#
# Message definitions
#

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    msg/DiskPartition.msg
    msg/NetworkAddress.msg
    msg/NetworkInterface.msg
    msg/PowerEvent.msg
    msg/PowerMode.msg
    msg/SerialDevice.msg
    msg/SerialDeviceScan.msg
    msg/SystemTelemetry.msg
    msg/UsbDevice.msg
    srv/PowerControl.srv
  DEPENDENCIES
    std_msgs
)

#
# Declare an ament Package
#

ament_package()

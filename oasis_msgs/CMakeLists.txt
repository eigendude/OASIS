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
    msg/AnalogReading.msg
    msg/AVRConstants.msg
    msg/CPUFanSpeed.msg
    msg/DigitalReading.msg
    msg/DiskPartition.msg
    msg/MCUMemory.msg
    msg/NetworkAddress.msg
    msg/NetworkInterface.msg
    msg/PeripheralConstants.msg
    msg/PeripheralInfo.msg
    msg/PeripheralScan.msg
    msg/PowerEvent.msg
    msg/PowerMode.msg
    msg/SerialDevice.msg
    msg/SerialDeviceScan.msg
    msg/SystemTelemetry.msg
    msg/UsbDevice.msg
    srv/AnalogRead.srv
    srv/DigitalRead.srv
    srv/DigitalWrite.srv
    srv/PowerControl.srv
    srv/PWMWrite.srv
    srv/ReportMCUMemory.srv
    srv/ServoWrite.srv
    srv/SetAnalogMode.srv
    srv/SetDigitalMode.srv
  DEPENDENCIES
    std_msgs
)

#
# Declare an ament Package
#

ament_package()

################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
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

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#
# Message definitions
#

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    msg/Accelerometer.msg
    msg/AirQuality.msg
    msg/AnalogButton.msg
    msg/AnalogReading.msg
    msg/AnalogStick.msg
    msg/AVRConstants.msg
    msg/CPUFanSpeed.msg
    msg/ConductorState.msg
    msg/DigitalButton.msg
    msg/DigitalReading.msg
    msg/DiskPartition.msg
    msg/EngineState.msg
    msg/I2CDevice.msg
    msg/I2CDeviceType.msg
    msg/I2CImu.msg
    msg/LabState.msg
    msg/LeonardoState.msg
    msg/MCUMemory.msg
    msg/MCUString.msg
    msg/NetworkAddress.msg
    msg/NetworkInterface.msg
    msg/PeripheralConstants.msg
    msg/PeripheralInfo.msg
    msg/PeripheralInput.msg
    msg/PeripheralScan.msg
    msg/PowerEvent.msg
    msg/PowerMode.msg
    msg/SerialDevice.msg
    msg/SerialDeviceScan.msg
    msg/SystemTelemetry.msg
    msg/Throttle.msg
    msg/UsbDevice.msg
    msg/Wheel.msg
    srv/AnalogRead.srv
    srv/CaptureInput.srv
    srv/DigitalRead.srv
    srv/DigitalWrite.srv
    srv/I2CBegin.srv
    srv/PowerControl.srv
    srv/PWMWrite.srv
    srv/ReportMCUMemory.srv
    srv/ServoWrite.srv
    srv/SetAnalogMode.srv
    srv/SetDigitalMode.srv
    srv/SetSamplingInterval.srv
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

#
# Declare an ament Package
#

ament_package()

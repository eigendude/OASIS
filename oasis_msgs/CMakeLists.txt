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
project(oasis_msgs)

#
# Dependencies
#

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

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
    msg/BoundingBox.msg
    msg/CameraScene.msg
    msg/ConductorState.msg
    msg/CPUFanSpeed.msg
    msg/DigitalButton.msg
    msg/DigitalReading.msg
    msg/DiskPartition.msg
    msg/EngineState.msg
    msg/I2CDevice.msg
    msg/I2CDeviceType.msg
    msg/I2CImu.msg
    msg/LabState.msg
    msg/Landmark.msg
    msg/LeonardoState.msg
    msg/MCUMemory.msg
    msg/MCUString.msg
    msg/MediaItem.msg
    msg/NetworkAddress.msg
    msg/NetworkInterface.msg
    msg/PeripheralConstants.msg
    msg/PeripheralInfo.msg
    msg/PeripheralInput.msg
    msg/PeripheralScan.msg
    msg/Plug.msg
    msg/PoseLandmarks.msg
    msg/PoseLandmarksArray.msg
    msg/PowerEvent.msg
    msg/PowerMode.msg
    msg/RGB.msg
    msg/SerialDevice.msg
    msg/SerialDeviceScan.msg
    msg/SystemTelemetry.msg
    msg/Throttle.msg
    msg/UPSStatus.msg
    msg/UsbDevice.msg
    msg/Wheel.msg
    srv/AnalogRead.srv
    srv/CaptureInput.srv
    srv/DigitalRead.srv
    srv/DigitalWrite.srv
    srv/GetMACAddress.srv
    srv/I2CBegin.srv
    srv/I2CEnd.srv
    srv/PowerControl.srv
    srv/PWMWrite.srv
    srv/ReportMCUMemory.srv
    srv/ServoWrite.srv
    srv/SetAnalogMode.srv
    srv/SetDigitalMode.srv
    srv/SetDisplay.srv
    srv/SetPlug.srv
    srv/SetRGB.srv
    srv/SetSamplingInterval.srv
    srv/UPSCommand.srv
    srv/WoLCommand.srv
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

#
# Declare an ament Package
#

ament_package()

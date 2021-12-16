################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

# Board configuration
if (NOT ARDUINO_BOARD_OPTIONS_FILE)
  set(ARDUINO_BOARD_OPTIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/BoardOptions.cmake")
endif()

#
# Arduino IDE download links
#
#   Linux 32 - https://downloads.arduino.cc/arduino-1.8.18-linux32.tar.xz
#   Linux 64 - https://downloads.arduino.cc/arduino-1.8.18-linux64.tar.xz
#   Linux ARM 32 - https://downloads.arduino.cc/arduino-1.8.18-linuxarm.tar.xz
#   Linux ARM 64 - https://downloads.arduino.cc/arduino-1.8.18-linuxaarch64.tar.xz
#   MacOS - https://downloads.arduino.cc/arduino-1.8.18-macosx.zip
#   Windows - https://downloads.arduino.cc/arduino-1.8.18-windows.zip
#
# TODO: Search /opt if not set on command line
#
if (NOT ARDUINO_INSTALL_PATH)
  set(ARDUINO_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/arduino-1.8.18")
endif()

#
# Load Arduino toolchain, requires git submodule:
#
#   git submodule init
#   git submodule update
#
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Arduino-CMake-Toolchain/Arduino-toolchain.cmake")

#
# Declare project
#
# This must be done after setting the CMake toolchain file
#
cmake_minimum_required(VERSION 3.10)
project(oasis_avr)

#
# Dependencies
#

include(ExternalProject)

find_package(ament_cmake REQUIRED)

# External projects
ExternalProject_Add(protothreads
  URL http://dunkels.com/adam/download/pt-1.4.tar.gz
  URL_HASH SHA256=eab53f3715a10b0d1eea70f4c456724095b64b89630aba26867a7c982723daa7
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#
# Declare an ament Package
#

ament_package()

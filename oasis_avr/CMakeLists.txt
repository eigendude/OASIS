################################################################################
#
#  Copyright (C) 2021 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.11)

################################################################################
#
# Arduino CMake toolchain configuration
#
# The advantage of using the Arduino CMake toolchain is that it provides wide
# board support and allows us to use first-party and third-party Arduino
# libraries.
#
# However, passing the toolchain file to Colcon would cause all packages to be
# built for AVR instead of just this one. Instead, we specify the toolchain file
# here. Critically, it must be specified before the first call to project().
#
# An external script, bootstrap.sh, is therefore need to put all Arduino
# dependencies in place. This script should be called before invoking Colcon.
#
################################################################################

#
# Board selection
#
# Current options are:
#
#   - uno
#   - leonardo
#

# TODO: Currently board selection is done by scanning the hostname, but is
# specific to my computer constellation. How to make this more generic?
cmake_host_system_information(RESULT HOST_NAME QUERY HOSTNAME)
message(STATUS "Configuring for hostname: ${HOST_NAME}")

if (HOST_NAME STREQUAL cinder)
  set(ARDUINO_VARIANT leonardo)
else()
  set(ARDUINO_VARIANT uno)
endif()

message(STATUS "Building for: ${ARDUINO_VARIANT}")

#
# Board configuration file
#

if (NOT ARDUINO_BOARD_OPTIONS_FILE)
  set(ARDUINO_BOARD_OPTIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/BoardOptions-${ARDUINO_VARIANT}.cmake")
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
# We can't add Arduino libraries the same way as normal dependencies. Arduino
# libraries have no CMakeLists.txt, and are defined with a "library.properties"
# file.
#
# Arduino libraries are found with the toolchain file, so the library search
# path needs to be set here.
#

list(APPEND ARDUINO_LIBRARIES_SEARCH_PATHS_EXTRA
  "${CMAKE_CURRENT_SOURCE_DIR}/libraries"
)

#
# Load Arduino toolchain
#

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Arduino-CMake-Toolchain/Arduino-toolchain.cmake")

################################################################################
#
# End Arduino CMake toolchain configuration
#
# Declare the project here. This must be done after setting the CMake toolchain
# file.
#
################################################################################

project(oasis_avr)

################################################################################
#
# Dependencies
#
################################################################################

find_package(ament_cmake REQUIRED)

################################################################################
#
# Declare an ament Package
#
################################################################################

ament_package()

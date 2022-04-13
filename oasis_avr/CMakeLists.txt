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

if (HOST_NAME STREQUAL station)
  # Building conductor
  set(ARDUINO_VARIANT uno)

  #
  # With everything disabled:
  # 6562 flash (20%), 647 data (31%)
  #
  # With diagnostics enabled:
  # 7104 flash (22%), 627 data (30%)
  # Heap: 885, stack: 301, free: 845
  #
  # With analog enabled:
  # 8628 flash (26%), 625 data (30%)
  # Heap: 883, stack: 426, free: 772
  #
  # With everything but external libraries enabled:
  # 9869 flash (30%), 630 data (30%)
  # Heap: 888, stack: 426, free: 777
  #
  # With CPU fan enabled:
  # 12232 flash (37%), 762 data (37%)
  # Heap: 1020, stack: 615, free: 396 (19%)
  #
  set(ENABLE_ANALOG ON)
  set(ENABLE_CPU_FAN ON)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
elseif (HOST_NAME STREQUAL jetson)
  # Building engine
  set(ARDUINO_VARIANT uno)

  #
  # With everything disabled:
  # 6944 flash (21%), 632 data (30%)
  #
  # With diagnostics enabled:
  # 7802 flash (24%), 668 data (32%)
  # Heap: 926, stack: 305, free: 800
  #
  # With analog enabled:
  # 8750 flash (27%), 694 data (33%)
  # Heap: 952, stack: 305, free: 774 (62%)
  #
  # With everything enabled except for subsystems with external libraries:
  # 12410 flash (38%), 773 data (37%)
  # Heap: 1031, stack: 305, free: 695 (66%)
  #
  set(ENABLE_ANALOG ON)
  set(ENABLE_CPU_FAN ON)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
elseif (HOSTNAME STREQUAL cinder)
  # Building leonardo
  set(ARDUINO_VARIANT leonardo)

  #
  # With external libraries disabled:
  # 13178 flash (45%), 744 data (29%)
  # Heap: 1002, stack: 490, free: 1051 (41%)
  #
  # With CPU fan enabled:
  # 14304 flash (49%), 746 data (29%)
  # Heap: 1004, stack: 615, free: 924 (36%)
  #
  # With Servo enabled:
  # 15146 flash (52%), 783 data (30%)
  # Heap: 1041, stack: 490, free: 1012 (39%)
  #
  # With SPI enabled:
  # 14728 flash (51%), 1147 data (44%)
  # Heap: 1405, stack: 490, free: 648 (25%)
  #
  # With Stepper enabled:
  # 15738 flash (54%), 924 data (36%)
  # Heap: 1182, stack: 490, free: 871 (34%)
  #
  # With everything enabled:
  # 25624 flash (89%), 1810 data (70%)
  #
  set(ENABLE_ANALOG ON)
  set(ENABLE_CPU_FAN OFF)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL OFF)
  set(ENABLE_I2C OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
else()
  # Building generic
  set(ARDUINO_VARIANT uno)

  set(ENABLE_ANALOG ON)
  set(ENABLE_CPU_FAN ON)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
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
  set(ARDUINO_INSTALL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/arduino-1.8.19")
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
# Build configuration
#
################################################################################

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_compile_definitions(
  ARDUINO_BOARD_RAM_SIZE=${ARDUINO_BOARD_RAM_SIZE}
)

################################################################################
#
# Dependencies
#
################################################################################

find_package(ament_cmake REQUIRED)

################################################################################
#
# C++ libraries
#
################################################################################

#
# Firmata
#

set(FIRMATA_SOURCES
  src/firmata/firmata_callbacks.cpp
  src/firmata/firmata_thread.cpp
  src/firmata/firmata_utils.cpp
)

set(FIRMATA_DEFINITIONS)

set(FIRMATA_LIBRARIES
  Adafruit_BluefruitLE_nRF51
  core
  FirmataExpress
  Scheduler
)

if (ENABLE_ANALOG)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_analog.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_ANALOG
  )
endif()

if (ENABLE_CPU_FAN)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_cpu_fan.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_CPU_FAN
  )
endif()

if (ENABLE_DHT)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_dht.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_DHT
  )
  list(APPEND FIRMATA_LIBRARIES
    DHTStable
  )
endif()

if (ENABLE_DIAGNOSTICS)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_diagnostics.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_DIAGNOSTICS
  )
endif()

if (ENABLE_DIGITAL)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_digital.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_DIGITAL
  )
endif()

if (ENABLE_I2C)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_i2c.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_I2C
  )
  list(APPEND FIRMATA_LIBRARIES
    Wire
  )
endif()

if (ENABLE_SERVO)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_servo.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_SERVO
  )
  list(APPEND FIRMATA_LIBRARIES
    Servo
  )
endif()

if (ENABLE_SONAR)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_sonar.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_SONAR
  )
  list(APPEND FIRMATA_LIBRARIES
    Ultrasonic
  )
endif()

if (ENABLE_SPI)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_spi.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_SPI
  )
  list(APPEND FIRMATA_LIBRARIES
    SPI
  )
endif()

if (ENABLE_STEPPER)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_stepper.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_STEPPER
  )
  list(APPEND FIRMATA_LIBRARIES
    Stepper
  )
endif()

add_library(oasis_avr_firmata ${FIRMATA_SOURCES})

target_compile_definitions(oasis_avr_firmata PRIVATE ${FIRMATA_DEFINITIONS})

target_link_arduino_libraries(oasis_avr_firmata PRIVATE ${FIRMATA_LIBRARIES})

#
# LEDs
#

add_library(oasis_avr_leds
  src/leds/heartbeat_thread.cpp
)

target_link_arduino_libraries(oasis_avr_leds PRIVATE
  core
  Scheduler
)

#
# Utils
#

add_library(oasis_avr_utils
  src/utils/delete.cpp
  src/utils/memory_utils.cpp
  src/utils/timer.cpp
)

target_link_arduino_libraries(oasis_avr_utils PRIVATE
  core
)

################################################################################
#
# C++ nodes
#
################################################################################

#
# firmata_node
#

add_executable(firmata_node
  src/ros/firmata_node.cpp
)

target_link_libraries(firmata_node PRIVATE
  oasis_avr_firmata
  oasis_avr_leds
  oasis_avr_utils
)

target_link_arduino_libraries(firmata_node PRIVATE ${FIRMATA_LIBRARIES})

# This is needed for the generation of HEX binary and uploading it
target_enable_arduino_upload(firmata_node)

install(
  TARGETS
    firmata_node
  DESTINATION
    share/${PROJECT_NAME}
)

################################################################################
#
# Declare an ament Package
#
################################################################################

ament_package()

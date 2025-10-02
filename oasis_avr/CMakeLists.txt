################################################################################
#
#  Copyright (C) 2021-2025 Garrett Brown
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
#   - mega
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
  # Telemetrix with no external libraries enabled:
  # 12380 flash (13%), 1036 data (50%)
  #
  set(ENABLE_AIR_QUALITY OFF)
  set(ENABLE_ANALOG ON)
  set(ENABLE_BLUEFRUIT OFF)
  set(ENABLE_CPU_FAN ON)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C OFF)
  set(ENABLE_IMU OFF)
  set(ENABLE_ONE_WIRE OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
elseif (HOST_NAME STREQUAL megapegasus)
  # Building lab
  set(ARDUINO_VARIANT uno)

  set(ENABLE_AIR_QUALITY ON)
  set(ENABLE_ANALOG ON)
  set(ENABLE_BLUEFRUIT OFF)
  set(ENABLE_CPU_FAN OFF)
  set(ENABLE_DHT OFF)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C ON)
  set(ENABLE_IMU ON)
  set(ENABLE_ONE_WIRE OFF)
  set(ENABLE_SERVO OFF)
  set(ENABLE_SONAR OFF)
  set(ENABLE_SPI OFF)
  set(ENABLE_STEPPER OFF)
else()
  set(ARDUINO_VARIANT mega)

  #
  # With external libraries disabled:
  # Program Size: 4632 of 32256 bytes (14%)
  # Data Size: 1062 of 2048 bytes (51%)
  # RAM: 54%
  #
  # With DHT:
  # Program Size: 7650 of 32256 bytes (23%)
  # Data Size: 1125 of 2048 bytes (54%)
  # RAM: 58%
  #
  # With I2C:
  # Program Size: 7382 of 32256 bytes (22%)
  # Data Size: 1354 of 2048 bytes (66%)
  # RAM: 68%
  #
  # With OneWire:
  # Program Size: 6724 of 32256 bytes (20%)
  # Data Size: 1076 of 2048 bytes (52%)
  # RAM: 54%
  #
  # With Servo:
  # Program Size: 6264 of 32256 bytes (19%)
  # Data Size: 1160 of 2048 bytes (56%)
  # RAM: 59%
  #
  # With Sonar:
  # Program Size: 5912 of 32256 bytes (18%)
  # Data Size: 1107 of 2048 bytes (54%)
  # RAM: 56%
  #
  # With SPI:
  # Program Size: 5020 of 32256 bytes (15%)
  # Data Size: 1127 of 2048 bytes (55%)
  # RAM: 57%
  #
  # With Stepper:
  # Program Size: 12238 of 32256 bytes (37%)
  # Data Size: 1100 of 2048 bytes (53%)
  # RAM: 56%
  #
  # With everything:
  # Program Size: 20986 of 32256 bytes (65%)
  # Data Size: 1707 of 2048 bytes (83%)
  # RAM: 85%
  #
  set(ENABLE_AIR_QUALITY ON)
  set(ENABLE_ANALOG ON)
  set(ENABLE_BLUEFRUIT ON)
  set(ENABLE_CPU_FAN ON)
  set(ENABLE_DHT ON)
  set(ENABLE_DIAGNOSTICS ON)
  set(ENABLE_DIGITAL ON)
  set(ENABLE_I2C ON)
  set(ENABLE_IMU ON)
  set(ENABLE_ONE_WIRE ON)
  set(ENABLE_SERVO ON)
  set(ENABLE_SONAR ON)
  set(ENABLE_SPI ON)
  set(ENABLE_STEPPER ON)
endif()

message(STATUS "Building for: ${ARDUINO_VARIANT}")

#
# Board configuration file
#

if (NOT ARDUINO_BOARD_OPTIONS_FILE)
  set(ARDUINO_BOARD_OPTIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/boards/BoardOptions-${ARDUINO_VARIANT}.cmake")
endif()

#
# Arduino IDE download links
#
#   Linux 32 - https://downloads.arduino.cc/arduino-1.8.19-linux32.tar.xz
#   Linux 64 - https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz
#   Linux ARM 32 - https://downloads.arduino.cc/arduino-1.8.19-linuxarm.tar.xz
#   Linux ARM 64 - https://downloads.arduino.cc/arduino-1.8.19-linuxaarch64.tar.xz
#   MacOS - https://downloads.arduino.cc/arduino-1.8.19-macosx.zip
#   Windows - https://downloads.arduino.cc/arduino-1.8.19-windows.zip
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

if (ENABLE_BLUEFRUIT)
  list(APPEND FIRMATA_SOURCES
    src/firmata/firmata_bluefruit.cpp
  )
  list(APPEND FIRMATA_DEFINITIONS
    -DENABLE_BLUEFRUIT
  )
  list(APPEND FIRMATA_LIBRARIES
    Adafruit_BluefruitLE_nRF51
    SPI
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
# Telemetrix
#

set(TELEMETRIX_SOURCES
  src/drivers/cpu_fan_pwm.cpp
  src/drivers/cpu_fan_tach.cpp
  src/telemetrix/telemetrix_commands.cpp
  src/telemetrix/telemetrix_cpu_fan.cpp
  src/telemetrix/telemetrix_memory.cpp
  src/telemetrix/telemetrix_pins.cpp
  src/telemetrix/telemetrix_server.cpp
  src/utils/delete.cpp
  src/utils/math_utils.cpp
  src/utils/memory_utils.cpp
  src/utils/timer.cpp
)

set(TELEMETRIX_DEFINITIONS)

set(TELEMETRIX_LIBRARIES
  core
  Scheduler
)

if (ENABLE_AIR_QUALITY)
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_AIR_QUALITY=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    SparkFunCCS811
    Wire
  )
endif()

if (ENABLE_DHT)
  list(APPEND TELEMETRIX_SOURCES
    src/drivers/dht.cpp
    src/telemetrix/telemetrix_dht.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_DHT=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    DHTStable
  )
endif()

if (ENABLE_I2C)
  list(APPEND TELEMETRIX_SOURCES
    src/drivers/i2c_port.cpp
    src/telemetrix/telemetrix_i2c.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_I2C=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    Wire
  )
endif()

if (ENABLE_IMU)
  include_directories(
    libraries/i2cdevlib/Arduino/I2Cdev
    libraries/i2cdevlib/Arduino/MPU6050
  )
  list(APPEND TELEMETRIX_SOURCES
    libraries/i2cdevlib/Arduino/I2Cdev/I2Cdev.cpp
    libraries/i2cdevlib/Arduino/MPU6050/MPU6050.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_IMU
  )
  list(APPEND TELEMETRIX_LIBRARIES
    Wire
  )
endif()

if (ENABLE_ONE_WIRE)
  list(APPEND TELEMETRIX_SOURCES
    src/telemetrix/telemetrix_one_wire.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_ONE_WIRE=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    OneWire
  )
endif()

if (ENABLE_SERVO)
  list(APPEND TELEMETRIX_SOURCES
    src/drivers/servomotor.cpp
    src/telemetrix/telemetrix_servo.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_SERVO=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    Servo
  )
endif()

if (ENABLE_SONAR)
  list(APPEND TELEMETRIX_SOURCES
    src/drivers/sonar.cpp
    src/telemetrix/telemetrix_sonar.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_SONAR=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    Ultrasonic
  )
endif()

if (ENABLE_SPI)
  list(APPEND TELEMETRIX_SOURCES
    src/telemetrix/telemetrix_spi.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_SPI=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    SPI
  )
endif()

if (ENABLE_STEPPER)
  list(APPEND TELEMETRIX_SOURCES
    src/telemetrix/telemetrix_stepper.cpp
  )
  list(APPEND TELEMETRIX_DEFINITIONS
    -DENABLE_STEPPER=1
  )
  list(APPEND TELEMETRIX_LIBRARIES
    AccelStepper
  )
endif()

add_library(oasis_avr_telemetrix ${TELEMETRIX_SOURCES})

target_compile_definitions(oasis_avr_telemetrix PRIVATE ${TELEMETRIX_DEFINITIONS})

target_link_arduino_libraries(oasis_avr_telemetrix PRIVATE ${TELEMETRIX_LIBRARIES})

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

target_link_arduino_libraries(firmata_node PRIVATE
  ${FIRMATA_LIBRARIES}
)

# This is needed for the generation of HEX binary and uploading it
#
# Upload with:
#   make upload TARGET=firmata_node SERIAL_PORT=<serial port, e.g. /dev/ttyACM0>
# e.g.
#   make -C "build/oasis_avr" upload TARGET=firmata_node SERIAL_PORT=/dev/ttyACM0
#
target_enable_arduino_upload(firmata_node)

install(
  TARGETS
    firmata_node
  DESTINATION
    share/${PROJECT_NAME}
)

#
# telemetrix_node
#

add_executable(telemetrix_node
  src/ros/telemetrix_node.cpp
)

target_compile_definitions(telemetrix_node PRIVATE
  ${TELEMETRIX_DEFINITIONS}
)

target_link_libraries(telemetrix_node PRIVATE
  oasis_avr_telemetrix
  oasis_avr_leds
)

target_link_arduino_libraries(telemetrix_node PRIVATE
  ${TELEMETRIX_LIBRARIES}
)

# This is needed for the generation of HEX binary and uploading it
#
# Upload with:
#   make upload TARGET=telemetrix_node SERIAL_PORT=<serial port, e.g. /dev/ttyACM0>
# e.g.
#   make -C "build/oasis_avr" upload TARGET=telemetrix_node SERIAL_PORT=/dev/ttyACM0
#
target_enable_arduino_upload(telemetrix_node)

install(
  TARGETS
    telemetrix_node
  DESTINATION
    share/${PROJECT_NAME}
)

################################################################################
#
# Install data files
#
################################################################################

install(
  DIRECTORY
    config/udev/
  DESTINATION
    share/${PROJECT_NAME}/udev
  FILES_MATCHING
    PATTERN "*.rules"
)

################################################################################
#
# Declare an ament Package
#
################################################################################

ament_package()

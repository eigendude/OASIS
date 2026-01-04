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
project(oasis_drivers_cpp)

################################################################################
# Build configuration
################################################################################

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# TODO: -Wpedantic causes warnings due to libcec
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

################################################################################
# Dependencies
################################################################################

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(CEC REQUIRED)
find_package(i2cdevlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(oasis_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(UDEV)

################################################################################
# C++ nodes
################################################################################

#
# cec_server
#

if (UDEV_FOUND)
  add_executable(cec_server
    src/cec/CecAdapter.cpp
    src/cec/CecServer.cpp
    src/cec/CecTranslator.cpp
    src/cec/CecUtils.cpp
    src/cli/cec_server.cpp
    src/nodes/CecServerNode.cpp
    src/udev/UdevScanner.cpp
    src/udev/UdevTranslator.cpp
    src/udev/UdevUtils.cpp
    src/utils/LogUtils.cpp
    src/utils/StringUtils.cpp
  )

  target_compile_definitions(cec_server PUBLIC
    ${CEC_DEFINITIONS}
    ${UDEV_DEFINITIONS}
  )

  target_include_directories(cec_server PUBLIC
    ${CEC_INCLUDE_DIRS}
    ${UDEV_INCLUDE_DIRS}
  )

  target_link_libraries(cec_server
    ${CEC_LIBRARIES}
    ${UDEV_LIBRARIES}
    ${oasis_msgs_TARGETS}
    rclcpp::rclcpp
    rcutils::rcutils
    ${std_msgs_TARGETS}
  )

  install(
    TARGETS
      cec_server
    DESTINATION
      lib/${PROJECT_NAME}
  )
endif()

#
# mpu6050_imu_driver
#

add_executable(mpu6050_imu_driver
  src/cli/mpu6050_imu_driver.cpp
  src/imu/cal/AccelCalibrationSolver.cpp
  src/imu/cal/GyroBiasEstimator.cpp
  src/imu/ImuProcessor.cpp
  src/imu/ImuTemperature.cpp
  src/imu/io/ImuCalibrationFile.cpp
  src/imu/Mpu6050ImuUtils.cpp
  src/imu/StationaryDetector.cpp
  src/nodes/Mpu6050Node.cpp
)

target_link_libraries(mpu6050_imu_driver
  i2cdevlib::mpu6050
  Eigen3::Eigen
  ${geometry_msgs_TARGETS}
  ${oasis_msgs_TARGETS}
  rclcpp::rclcpp
  rcutils::rcutils
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  yaml-cpp
)

install(
  TARGETS
    mpu6050_imu_driver
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# mmc5983ma_magnetometer_driver
#

add_executable(mmc5983ma_magnetometer_driver
  src/cli/mmc5983ma_magnetometer_driver.cpp
  src/magnetometer/CovarianceEstimator3D.cpp
  src/magnetometer/Mmc5983maDevice.cpp
  src/magnetometer/io/MagnetometerCalibrationFile.cpp
  src/nodes/Mmc5983maMagnetometerNode.cpp
)

target_link_libraries(mmc5983ma_magnetometer_driver
  i2cdevlib::mpu6050
  Eigen3::Eigen
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  yaml-cpp
)

install(
  TARGETS
    mmc5983ma_magnetometer_driver
  DESTINATION
    lib/${PROJECT_NAME}
)

################################################################################
# Configuration files
################################################################################

install(
  DIRECTORY
    config/imu_info
  DESTINATION
    share/${PROJECT_NAME}/imu_info
)

################################################################################
# Testing
################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Declare an ament Package
################################################################################

ament_package()

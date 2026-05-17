################################################################################
#
#  Copyright (C) 2021-2026 Garrett Brown
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
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(i2cdevlib REQUIRED)
find_package(oasis_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(UDEV)
find_package(yaml-cpp REQUIRED)

################################################################################
# C++ nodes
################################################################################

#
# bno086_imu_driver
#

add_library(bno086_drain_policy
  src/imu/bno086/core/Bno086DrainPolicy.cpp
)

add_library(bno086_gravity_utils
  src/imu/bno086/core/Bno086GravityUtils.cpp
)

add_library(bno086_imu_gravity_accel_history
  src/imu/bno086/core/Bno086ImuGravityAccelHistory.cpp
)

add_library(bno086_orientation_covariance_policy
  src/imu/bno086/core/Bno086OrientationCovariancePolicy.cpp
)

add_library(bno086_report_timestamp_tracker
  src/imu/bno086/core/Bno086ReportTimestampTracker.cpp
)

add_library(bno086_sample_coherence
  src/imu/bno086/core/Bno086SampleCoherence.cpp
)

add_library(bno086_timestamp_cadence
  src/imu/bno086/core/Bno086TimestampCadence.cpp
)
target_link_libraries(bno086_timestamp_cadence
  bno086_report_timestamp_tracker
)

add_library(bno086_utils
  src/imu/bno086/utils/Bno086CovarianceUtils.cpp
  src/imu/bno086/utils/Bno086FeatureSummary.cpp
  src/imu/bno086/utils/Bno086MathUtils.cpp
  src/imu/bno086/utils/Bno086ReportUtils.cpp
  src/imu/bno086/utils/Bno086TimingUtils.cpp
)

add_library(bno086_diagnostics
  src/imu/bno086/diagnostics/Bno086DrainHealth.cpp
  src/imu/bno086/diagnostics/Bno086RateHealth.cpp
)
target_link_libraries(bno086_diagnostics
  bno086_drain_policy
  bno086_utils
)

add_library(bno086_shtp
  src/imu/bno086/shtp/Bno086ShtpHeader.cpp
  src/imu/bno086/shtp/Bno086Transport.cpp
)

add_library(bno086_sh2
  src/imu/bno086/sh2/Bno086Shtp.cpp
)
target_link_libraries(bno086_sh2
  bno086_shtp
)

add_executable(bno086_imu_driver
  src/cli/bno086_imu_driver.cpp
  src/imu/bno086/gpio/Bno086Gpio.cpp
  src/nodes/Bno086ImuNode.cpp
)

target_link_libraries(bno086_imu_driver
  bno086_diagnostics
  bno086_drain_policy
  bno086_gravity_utils
  bno086_imu_gravity_accel_history
  bno086_orientation_covariance_policy
  bno086_report_timestamp_tracker
  bno086_sample_coherence
  bno086_sh2
  bno086_shtp
  bno086_timestamp_cadence
  bno086_utils
  ${geometry_msgs_TARGETS}
  ${oasis_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(
  TARGETS
    bno086_imu_driver
  DESTINATION
    lib/${PROJECT_NAME}
)

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
# mmc5983ma_magnetometer_driver
#

add_executable(mmc5983ma_magnetometer_driver
  src/cli/mmc5983ma_magnetometer_driver.cpp
  src/magnetometer/CovarianceEstimator3D.cpp
  src/magnetometer/Mmc5983maDevice.cpp
  src/magnetometer/IMagnetometerPairSampler.h
  src/magnetometer/MagnetometerPairProcessor.cpp
  src/magnetometer/MagnetometerPairProcessor.h
  src/magnetometer/Mmc5983maPairSampler.cpp
  src/magnetometer/Mmc5983maPairSampler.h
  src/magnetometer/io/MagnetometerCalibrationFile.cpp
  src/nodes/Mmc5983maMagnetometerNode.cpp
)

target_link_libraries(mmc5983ma_magnetometer_driver
  i2cdevlib::i2cdevlib
  Eigen3::Eigen
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  yaml-cpp
)

install(
  TARGETS
    mmc5983ma_magnetometer_driver
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# mpu6050_imu_driver
#

add_executable(mpu6050_imu_driver
  src/cli/mpu6050_imu_driver.cpp
  src/imu/mpu6050/cal/AccelCalibrationSolver.cpp
  src/imu/mpu6050/cal/GyroBiasEstimator.cpp
  src/imu/mpu6050/io/ImuCalibrationFile.cpp
  src/imu/mpu6050/Mpu6050ImuProcessor.cpp
  src/imu/mpu6050/Mpu6050ImuTemperature.cpp
  src/imu/mpu6050/Mpu6050ImuUtils.cpp
  src/imu/mpu6050/StationaryDetector.cpp
  src/nodes/Mpu6050Node.cpp
)

target_link_libraries(mpu6050_imu_driver
  i2cdevlib::i2cdevlib
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

################################################################################
# Configuration files
################################################################################

install(
  DIRECTORY
    config/imu_info
  DESTINATION
    share/${PROJECT_NAME}/imu_info
)

install(
  DIRECTORY
    config/magnetometer_info
  DESTINATION
    share/${PROJECT_NAME}/magnetometer_info
)

################################################################################
# Testing
################################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_add_gtest(test_bno086_gravity_utils
    test/imu/bno086/test_Bno086GravityUtils.cpp
  )
  target_link_libraries(test_bno086_gravity_utils
    bno086_gravity_utils
  )

  ament_add_gtest(test_bno086_orientation_covariance_policy
    test/imu/bno086/test_Bno086OrientationCovariancePolicy.cpp
  )
  target_link_libraries(test_bno086_orientation_covariance_policy
    bno086_orientation_covariance_policy
  )

  ament_add_gtest(test_bno086_drain_policy
    test/imu/bno086/test_Bno086DrainPolicy.cpp
  )
  target_link_libraries(test_bno086_drain_policy
    bno086_drain_policy
    bno086_sh2
    bno086_shtp
  )

  ament_add_gtest(test_bno086_drain_health
    test/imu/bno086/test_Bno086DrainHealth.cpp
  )
  target_link_libraries(test_bno086_drain_health
    bno086_diagnostics
    bno086_drain_policy
  )

  ament_add_gtest(test_bno086_imu_gravity_accel_history
    test/imu/bno086/test_Bno086ImuGravityAccelHistory.cpp
  )
  target_link_libraries(test_bno086_imu_gravity_accel_history
    bno086_imu_gravity_accel_history
  )

  ament_add_gtest(test_bno086_rate_health
    test/imu/bno086/test_Bno086RateHealth.cpp
  )
  target_link_libraries(test_bno086_rate_health
    bno086_diagnostics
    bno086_utils
  )

  ament_add_gtest(test_bno086_shtp
    test/imu/bno086/test_Bno086Shtp.cpp
  )
  target_link_libraries(test_bno086_shtp
    bno086_sh2
    bno086_shtp
  )

  ament_add_gtest(test_bno086_transport
    test/imu/bno086/test_Bno086Transport.cpp
  )
  target_link_libraries(test_bno086_transport
    bno086_shtp
  )

  ament_add_gtest(test_bno086_report_timestamp_tracker
    test/imu/bno086/test_Bno086ReportTimestampTracker.cpp
  )
  target_link_libraries(test_bno086_report_timestamp_tracker
    bno086_report_timestamp_tracker
  )

  ament_add_gtest(test_bno086_sample_coherence
    test/imu/bno086/test_Bno086SampleCoherence.cpp
  )
  target_link_libraries(test_bno086_sample_coherence
    bno086_sample_coherence
  )

  ament_add_gtest(test_bno086_timestamp_cadence
    test/imu/bno086/test_Bno086TimestampCadence.cpp
  )
  target_link_libraries(test_bno086_timestamp_cadence
    bno086_timestamp_cadence
    bno086_report_timestamp_tracker
  )

  ament_add_gtest(test_bno086_utils
    test/imu/bno086/test_Bno086Utils.cpp
  )
  target_link_libraries(test_bno086_utils
    bno086_utils
  )

  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Declare an ament Package
################################################################################

ament_package()

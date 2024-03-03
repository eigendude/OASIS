################################################################################
#
#  Copyright (C) 2021-2024 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

cmake_minimum_required(VERSION 3.5)
project(oasis_perception)

#
# Build configuration
#

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenGL_GL_PREFERENCE GLVND)

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(bgslibrary REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(oasis_msgs REQUIRED)
find_package(ORB_SLAM3)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if (ORB_SLAM3_FOUND)
  find_package(DBoW2 REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(g2o REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(Sophus REQUIRED)
endif()

#
# background_subtractor_abl
#

add_executable(background_subtractor_abl
  src/image/BackgroundModelerABL.cpp
  src/utils/NetworkUtils.cpp
  src/background_subtractor_abl.cpp
)

ament_target_dependencies(background_subtractor_abl
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

target_include_directories(background_subtractor_abl PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor_abl
  ${BGSLIBRARY_LIBRARIES}
)

install(
  TARGETS
    background_subtractor_abl
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# background_subtractor_asbl
#

add_executable(background_subtractor_asbl
  src/image/BackgroundModelerASBL.cpp
  src/utils/NetworkUtils.cpp
  src/background_subtractor_asbl.cpp
)

ament_target_dependencies(background_subtractor_asbl
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

target_include_directories(background_subtractor_asbl PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor_asbl
  ${BGSLIBRARY_LIBRARIES}
)

install(
  TARGETS
    background_subtractor_asbl
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# monocular_slam
#

if (ORB_SLAM3_FOUND)
  add_executable(monocular_slam
    src/slam/MonocularSlam.cpp
    src/utils/NetworkUtils.cpp
    src/monocular_slam.cpp
  )

  ament_target_dependencies(monocular_slam
    cv_bridge
    image_transport
    rclcpp
    sensor_msgs
  )

  target_compile_definitions(monocular_slam PUBLIC
    HAVE_EIGEN
    HAVE_GLEW
  )

  target_include_directories(monocular_slam PUBLIC
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLEW_INCLUDE_DIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${SOPHUS_INCLUDE_DIRS}
  )

  target_link_libraries(monocular_slam
    ${DBoW2_LIBRARIES}
    ${g2o_LIBRARIES}
    ${GLEW_LIBRARY}
    ${ORB_SLAM3_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${Pangolin_LIBRARIES}
  )

  install(
    TARGETS
      monocular_slam
    DESTINATION
      lib/${PROJECT_NAME}
  )
endif()

#
# monocular_inertial_slam
#

if (ORB_SLAM3_FOUND)
  add_executable(monocular_inertial_slam
    src/slam/MonocularInertialSlam.cpp
    src/utils/NetworkUtils.cpp
    src/monocular_inertial_slam.cpp
  )

  ament_target_dependencies(monocular_inertial_slam
    cv_bridge
    geometry_msgs
    image_transport
    oasis_msgs
    rclcpp
    sensor_msgs
  )

  target_compile_definitions(monocular_inertial_slam PUBLIC
    HAVE_EIGEN
    HAVE_GLEW
  )

  target_include_directories(monocular_inertial_slam PUBLIC
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLEW_INCLUDE_DIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${SOPHUS_INCLUDE_DIRS}
  )

  target_link_libraries(monocular_inertial_slam
    ${DBoW2_LIBRARIES}
    ${g2o_LIBRARIES}
    ${GLEW_LIBRARY}
    ${ORB_SLAM3_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${Pangolin_LIBRARIES}
  )

  install(
    TARGETS
      monocular_inertial_slam
    DESTINATION
      lib/${PROJECT_NAME}
  )
endif()

#
# multi_modeler
#

add_executable(multi_modeler
  src/image/MultiModeler.cpp
  src/utils/NetworkUtils.cpp
  src/multi_modeler.cpp
)

ament_target_dependencies(multi_modeler
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

target_include_directories(multi_modeler PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(multi_modeler
  ${BGSLIBRARY_LIBRARIES}
)

install(
  TARGETS
    multi_modeler
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# Testing
#

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Copyright and license must be present in all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # The following line skips cpplint (only works in a git repo)
  # Uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

#
# Launch files
#

install(
  FILES
    launch/perception_launch.py
  DESTINATION
    share/${PROJECT_NAME}
)

#
# Systemd services
#

install(
  FILES
    config/systemd/oasis_perception.service
  DESTINATION
    share/${PROJECT_NAME}/systemd
)

#
# Declare an ament Package
#

ament_package()

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
project(oasis_perception_cpp)

#
# Build configuration
#

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenGL_GL_PREFERENCE GLVND)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(bgslibrary REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(MediaPipe REQUIRED)
find_package(oasis_msgs REQUIRED)
find_package(ORB_SLAM3)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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

# Common dependencies for executables
set(AMENT_COMMON_DEPENDENCIES
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
)

# bgslibrary dependencies to link in
set(BGS_LIBRARIES
  ${BGSLIBRARY_LIBRARIES}
)

# TODO: Required by bgslibrary
find_package(OpenCV REQUIRED)
list(APPEND BGS_LIBRARIES ${OpenCV_LIBS})

# Sources needed for MediaPipe
set(MEDIAPIPE_SOURCES
  src/utils/AbseilStubs.cpp
)

#
# background_subtractor_abl
#

add_executable(background_subtractor_abl
  src/image/BackgroundModelerABL.cpp
  src/utils/NetworkUtils.cpp
  src/background_subtractor_abl.cpp
)

ament_target_dependencies(background_subtractor_abl
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_subtractor_abl PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor_abl
  ${BGS_LIBRARIES}
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
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_subtractor_asbl PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor_asbl
  ${BGS_LIBRARIES}
)

install(
  TARGETS
    background_subtractor_asbl
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# background_modeler
#

add_executable(background_modeler
  src/cli/background_modeler.cpp
  src/nodes/BackgroundModelerNode.cpp
)

ament_target_dependencies(background_modeler
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_modeler PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_modeler
  ${BGS_LIBRARIES}
)

install(
  TARGETS
    background_modeler
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# hello_world
#

set(HELLO_WORLD_SOURCES
  ${MEDIAPIPE_SOURCES}
  src/nodes/HelloWorldNode.cpp
  src/utils/HelloWorld.cpp
)
set(HELLO_WORLD_LIBRARIES
  MediaPipe::MediaPipe
  rclcpp::rclcpp
)

add_executable(hello_world
  ${HELLO_WORLD_SOURCES}
  src/cli/hello_world_node.cpp
)
target_link_libraries(hello_world PUBLIC
  ${HELLO_WORLD_LIBRARIES}
)
install(
  TARGETS
    hello_world
  DESTINATION
    lib/${PROJECT_NAME}
)

add_library(hello_world_component SHARED
  ${HELLO_WORLD_SOURCES}
  src/components/hello_world_component.cpp
  src/ros/RosComponent.cpp
)
target_include_directories(hello_world_component PUBLIC
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(hello_world_component PUBLIC
  ${HELLO_WORLD_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)
install(
  TARGETS
    hello_world_component
  DESTINATION
    lib
)

# Register the component
rclcpp_components_register_node(
  hello_world_component
  PLUGIN "oasis_perception::HelloWorldComponent"
  EXECUTABLE hello_world_component_node
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

ament_target_dependencies(multi_modeler ${AMENT_COMMON_DEPENDENCIES})

target_include_directories(multi_modeler PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(multi_modeler
  ${BGS_LIBRARIES}
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
# Declare an ament Package
#

ament_package()

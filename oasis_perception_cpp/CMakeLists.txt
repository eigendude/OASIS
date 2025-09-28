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

# Policy CMP0072: Prefer GLVND for OpenGL over legacy GL library
set(OpenGL_GL_PREFERENCE GLVND)

# Policy CMP0074: Ensure find_package honors *_ROOT hints so we pick up our
# custom OpenCV toolchain
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Load OpenCV variables from the environment
if(DEFINED ENV{OpenCV_DIR})
  set(OpenCV_DIR $ENV{OpenCV_DIR} CACHE PATH "Path to OpenCVConfig.cmake")
endif()
if(DEFINED ENV{OpenCV_ROOT})
  set(OpenCV_ROOT $ENV{OpenCV_ROOT} CACHE PATH "Path to OpenCV installation")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(bgslibrary REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(MediaPipe)
find_package(oasis_msgs REQUIRED)
find_package(ORB_SLAM3)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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
# background_modeler
#

# Project files
set(BACKGROUND_MODELER_SOURCES
  src/image/BackgroundModelerABL.cpp
  src/nodes/BackgroundModelerNode.cpp
)
set(BACKGROUND_MODELER_INCLUDE_DIRS
  ${BGSLIBRARY_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
set(BACKGROUND_MODELER_LIBRARIES
  ${BGS_LIBRARIES}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Node
add_executable(background_modeler
  ${BACKGROUND_MODELER_SOURCES}
  src/cli/background_modeler.cpp
)
target_include_directories(background_modeler PUBLIC
  ${BACKGROUND_MODELER_INCLUDE_DIRS}
)
target_link_libraries(background_modeler PUBLIC
  ${BACKGROUND_MODELER_LIBRARIES}
)
install(
  TARGETS
    background_modeler
  DESTINATION
    lib/${PROJECT_NAME}
)

# Component
add_library(background_modeler_component SHARED
  ${BACKGROUND_MODELER_SOURCES}
  src/components/background_modeler_component.cpp
  src/ros/RosComponent.cpp
)
target_include_directories(background_modeler_component PUBLIC
  ${BACKGROUND_MODELER_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(background_modeler_component PUBLIC
  ${BACKGROUND_MODELER_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)
install(
  TARGETS
    background_modeler_component
  DESTINATION
    lib
)
rclcpp_components_register_node(
  background_modeler_component
  PLUGIN "oasis_perception_cpp::BackgroundModelerComponent"
  EXECUTABLE background_modeler_component_node
)

#
# background_subtractor
#

# Project files
set(BACKGROUND_SUBTRACTOR_SOURCES
  src/image/BackgroundSubtractorASBL.cpp
  src/nodes/BackgroundSubtractorNode.cpp
)
set(BACKGROUND_SUBTRACTOR_INCLUDE_DIRS
  ${BGSLIBRARY_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
set(BACKGROUND_SUBTRACTOR_LIBRARIES
  ${BGS_LIBRARIES}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Node
add_executable(background_subtractor
  ${BACKGROUND_SUBTRACTOR_SOURCES}
  src/cli/background_subtractor.cpp
)
target_include_directories(background_subtractor PUBLIC
  ${BACKGROUND_SUBTRACTOR_INCLUDE_DIRS}
)
target_link_libraries(background_subtractor PUBLIC
  ${BACKGROUND_SUBTRACTOR_LIBRARIES}
)
install(
  TARGETS
    background_subtractor
  DESTINATION
    lib/${PROJECT_NAME}
)

# Component
add_library(background_subtractor_component SHARED
  ${BACKGROUND_SUBTRACTOR_SOURCES}
  src/components/background_subtractor_component.cpp
  src/ros/RosComponent.cpp
)
target_include_directories(background_subtractor_component PUBLIC
  ${BACKGROUND_SUBTRACTOR_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(background_subtractor_component PUBLIC
  ${BACKGROUND_SUBTRACTOR_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)
install(
  TARGETS
    background_subtractor_component
  DESTINATION
    lib
)
rclcpp_components_register_node(
  background_subtractor_component
  PLUGIN "oasis_perception_cpp::BackgroundSubtractorComponent"
  EXECUTABLE background_subtractor_component_node
)

#
# hello_world
#

if(TARGET MediaPipe::MediaPipe)
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
endif()

#
# monocular_slam
#

if (ORB_SLAM3_FOUND)
  # Project files
  set(MONOCULAR_SLAM_SOURCES
    src/slam/MonocularSlam.cpp
    src/utils/NetworkUtils.cpp
  )
  set(MONOCULAR_SLAM_DEFINITIONS
    HAVE_EIGEN
    HAVE_GLEW
  )
  set(MONOCULAR_SLAM_INCLUDE_DIRS
    $<BUILD_INTERFACE:${OPENGL_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${GLEW_INCLUDE_DIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${SOPHUS_INCLUDE_DIRS}
  )
  set(MONOCULAR_SLAM_LIBRARIES
    cv_bridge::cv_bridge
    ${DBoW2_LIBRARIES}
    ${g2o_LIBRARIES}
    ${GLEW_LIBRARY}
    image_transport::image_transport
    ${ORB_SLAM3_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${Pangolin_LIBRARIES}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )

  # Node
  add_executable(monocular_slam
    ${MONOCULAR_SLAM_SOURCES}
    src/monocular_slam.cpp
  )
  target_compile_definitions(monocular_slam PUBLIC
    ${MONOCULAR_SLAM_DEFINITIONS}
  )
  target_include_directories(monocular_slam PUBLIC
    ${MONOCULAR_SLAM_INCLUDE_DIRS}
  )
  target_link_libraries(monocular_slam PUBLIC
    ${MONOCULAR_SLAM_LIBRARIES}
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
  # Project files
  set(MONOCULAR_INERTIAL_SLAM_SOURCES
    src/slam/MonocularInertialSlam.cpp
    src/utils/NetworkUtils.cpp
  )
  set(MONOCULAR_INERTIAL_SLAM_DEFINITIONS
    ${MONOCULAR_SLAM_DEFINITIONS}
  )
  set(MONOCULAR_INERTIAL_SLAM_INCLUDE_DIRS
    ${MONOCULAR_SLAM_INCLUDE_DIRS}
  )
  set(MONOCULAR_INERTIAL_SLAM_LIBRARIES
    ${MONOCULAR_SLAM_LIBRARIES}
    ${geometry_msgs_TARGETS}
    ${oasis_msgs_TARGETS}
  )

  # Node
  add_executable(monocular_inertial_slam
    ${MONOCULAR_INERTIAL_SLAM_SOURCES}
    src/monocular_inertial_slam.cpp
  )
  target_compile_definitions(monocular_inertial_slam PUBLIC
    ${MONOCULAR_INERTIAL_SLAM_DEFINITIONS}
  )
  target_include_directories(monocular_inertial_slam PUBLIC
    ${MONOCULAR_INERTIAL_SLAM_INCLUDE_DIRS}
  )
  target_link_libraries(monocular_inertial_slam
    ${MONOCULAR_INERTIAL_SLAM_LIBRARIES}
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

# Project files
set(MULTI_MODELER_SOURCES
  src/image/MultiModeler.cpp
  src/utils/NetworkUtils.cpp
)
set(MULTI_MODELER_INCLUDE_DIRS
  ${BGSLIBRARY_INCLUDE_DIRS}
)
set(MULTI_MODELER_LIBRARIES
  ${BGS_LIBRARIES}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Node
add_executable(multi_modeler
  ${MULTI_MODELER_SOURCES}
  src/multi_modeler.cpp
)
target_include_directories(multi_modeler PUBLIC
  ${MULTI_MODELER_INCLUDE_DIRS}
)
target_link_libraries(multi_modeler
  ${MULTI_MODELER_LIBRARIES}
)
install(
  TARGETS
    multi_modeler
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# optical_flow
#

# Project files
set(OPTICAL_FLOW_SOURCES
  src/api/ImageProc.cpp
  src/api/Video.cpp
  src/kernels/cpu/CPUImageProc.cpp
  src/kernels/cpu/CPUVideo.cpp
  src/nodes/OpticalFlowNode.cpp
  src/utils/MathUtils.cpp
  src/video/OpticalFlow.cpp
  src/video/VisionGraph.cpp
)
set(OPTICAL_FLOW_INCLUDE_DIRS
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
set(OPTICAL_FLOW_LIBRARIES
  ${OpenCV_LIBS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# Node
add_executable(optical_flow
  ${OPTICAL_FLOW_SOURCES}
  src/cli/optical_flow_node.cpp
)
target_include_directories(optical_flow PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(optical_flow PUBLIC
  ${OPTICAL_FLOW_LIBRARIES}
)
install(
  TARGETS
    optical_flow
  DESTINATION
    lib/${PROJECT_NAME}
)

# Component
add_library(optical_flow_component SHARED
  ${OPTICAL_FLOW_SOURCES}
  src/components/optical_flow_component.cpp
  src/ros/RosComponent.cpp
)
target_include_directories(optical_flow_component PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)
target_link_libraries(optical_flow_component PUBLIC
  ${OPTICAL_FLOW_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)
install(
  TARGETS
    optical_flow_component
  DESTINATION
    lib
)
rclcpp_components_register_node(
  optical_flow_component
  PLUGIN "oasis_perception::OpticalFlowComponent"
  EXECUTABLE optical_flow_component_node
)

#
# pose_landmarker
#

# Check if the target MediaPipe::MediaPipe is available
if(TARGET MediaPipe::MediaPipe)
  # Project files
  set(POSE_LANDMARKER_SOURCES
    ${MEDIAPIPE_SOURCES}
    src/nodes/PoseLandmarkerNode.cpp
    src/pose/PoseLandmarker.cpp
  )
  set(POSE_LANDMARKER_LIBRARIES
    cv_bridge::cv_bridge
    image_transport::image_transport
    MediaPipe::MediaPipe
    ${oasis_msgs_TARGETS}
    ${OpenCV_LIBRARIES}
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
  )

  # Node
  add_executable(pose_landmarker
    ${POSE_LANDMARKER_SOURCES}
    src/cli/pose_landmarker_node.cpp
  )
  target_link_libraries(pose_landmarker PUBLIC
    ${POSE_LANDMARKER_LIBRARIES}
  )
  install(
    TARGETS
      pose_landmarker
    DESTINATION
      lib/${PROJECT_NAME}
  )

  # Component
  add_library(pose_landmarker_component SHARED
    ${POSE_LANDMARKER_SOURCES}
    src/components/pose_landmarker_component.cpp
    src/ros/RosComponent.cpp
  )
  target_include_directories(pose_landmarker_component PUBLIC
    ${rclcpp_components_INCLUDE_DIRS}
  )
  target_link_libraries(pose_landmarker_component PUBLIC
    ${POSE_LANDMARKER_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
  )
  install(
    TARGETS
      pose_landmarker_component
    DESTINATION
      lib
  )
  rclcpp_components_register_node(
    pose_landmarker_component
    PLUGIN "pose_landmarker::PoseLandmarkerComponent"
    EXECUTABLE pose_landmarker_component_node
  )
endif()

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

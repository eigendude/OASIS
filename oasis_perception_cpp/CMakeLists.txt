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

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

#
# Dependencies
#

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(bgslibrary REQUIRED)
find_package(cv_bridge REQUIRED)
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

add_library(background_modeler_component SHARED
  src/nodes/BackgroundModelerNode.cpp
  src/components/background_modeler_component.cpp
  src/ros/RosComponent.cpp
)

ament_target_dependencies(background_modeler_component
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_modeler_component PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

target_link_libraries(background_modeler_component
  ${BGS_LIBRARIES}
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

add_executable(background_subtractor
  src/cli/background_subtractor.cpp
  src/nodes/BackgroundSubtractorNode.cpp
)

ament_target_dependencies(background_subtractor
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_subtractor PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor
  ${BGS_LIBRARIES}
)

install(
  TARGETS
    background_subtractor
  DESTINATION
    lib/${PROJECT_NAME}
)

add_library(background_subtractor_component SHARED
  src/nodes/BackgroundSubtractorNode.cpp
  src/components/background_subtractor_component.cpp
  src/ros/RosComponent.cpp
)

ament_target_dependencies(background_subtractor_component
  ${AMENT_COMMON_DEPENDENCIES}
)

target_include_directories(background_subtractor_component PUBLIC
  ${BGSLIBRARY_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

target_link_libraries(background_subtractor_component
  ${BGS_LIBRARIES}
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
# optical_flow
#

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
set(OPTICAL_FLOW_LIBRARIES
  ${OpenCV_LIBS}
  rclcpp::rclcpp
)

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

# Register the component
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
  set(POSE_LANDMARKER_SOURCES
    ${MEDIAPIPE_SOURCES}
    src/nodes/PoseLandmarkerNode.cpp
    src/pose/PoseLandmarker.cpp
  )
  set(POSE_LANDMARKER_INCLUDE_DIRS
    ${oasis_msgs_INCLUDE_DIRS}
  )
  set(POSE_LANDMARKER_LIBRARIES
    cv_bridge::cv_bridge
    image_transport::image_transport
    MediaPipe::MediaPipe
    oasis_msgs::oasis_msgs__rosidl_generator_cpp
    ${OpenCV_LIBRARIES}
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    std_msgs::std_msgs__rosidl_generator_cpp
  )

  add_executable(pose_landmarker
    ${POSE_LANDMARKER_SOURCES}
    src/cli/pose_landmarker_node.cpp
  )
  target_include_directories(pose_landmarker PUBLIC
    ${POSE_LANDMARKER_INCLUDE_DIRS}
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

  add_library(pose_landmarker_component SHARED
    ${POSE_LANDMARKER_SOURCES}
    src/components/pose_landmarker_component.cpp
    src/ros/RosComponent.cpp
  )
  target_include_directories(pose_landmarker_component PUBLIC
    ${POSE_LANDMARKER_INCLUDE_DIRS}
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

  # Register the component
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

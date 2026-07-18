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
project(oasis_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMP0074 NEW makes find_package honor <PackageName>_ROOT variables; OASIS's
# env_cv.sh sets OpenCV_ROOT to select the repository-managed OpenCV build
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

#
# Dependencies
#

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

#
# OLED visualizer targets
#
# The dependency layers are renderer -> node -> component/CLI
#

# oled_renderer shared library
add_library(oled_renderer SHARED
    src/oled/oled_visualizer.cpp
)
target_include_directories(oled_renderer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/oasis_visualization>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(oled_renderer PUBLIC
  ${OpenCV_LIBRARIES}
)

# oled_visualizer_node shared library
add_library(oled_visualizer_node SHARED
  src/nodes/oled_visualizer_node.cpp
)
target_include_directories(oled_visualizer_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/oasis_visualization>
)
target_link_libraries(oled_visualizer_node PUBLIC
  oled_renderer
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

# oled_visualizer_component shared library
add_library(oled_visualizer_component SHARED
  src/components/oled_visualizer_component.cpp
)
target_link_libraries(oled_visualizer_component PUBLIC
  oled_visualizer_node
  rclcpp_components::component
)
rclcpp_components_register_nodes(oled_visualizer_component
  "OASIS::Visualization::OledVisualizerNode"
)

# oled_visualizer executable
add_executable(oled_visualizer
    src/cli/oled_visualizer.cpp
)
target_link_libraries(oled_visualizer PRIVATE
    oled_visualizer_node
)

# Installation
install(
  TARGETS
    oled_renderer
    oled_visualizer_node
    oled_visualizer
    oled_visualizer_component
  EXPORT oasis_visualizationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES src/oled/oled_visualizer.hpp
        DESTINATION include/oasis_visualization/oled)
install(FILES src/nodes/oled_visualizer_node.hpp
        DESTINATION include/oasis_visualization/nodes)

# Ament exports
ament_export_targets(oasis_visualizationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV
    rclcpp rclcpp_components
    sensor_msgs
)

#
# Tests
#

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_oled_visualizer
    test/oled/test_oled_visualizer.cpp
  )
  target_link_libraries(test_oled_visualizer oled_renderer)
endif()

#
# Scripts
#

install(
  PROGRAMS
    scripts/dismiss-osd.sh
  DESTINATION
    lib/${PROJECT_NAME}
)

#
# Configuration files
#

install(
  DIRECTORY
    config/systemd/
  DESTINATION
    share/${PROJECT_NAME}/systemd
)

#
# OLED graphics
#

install(
  DIRECTORY
    media/oled/
  DESTINATION
    share/${PROJECT_NAME}/media/oled
)

#
# Declare an ament Package
#

ament_package()

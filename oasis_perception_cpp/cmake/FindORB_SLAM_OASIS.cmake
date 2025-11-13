#.rst:
# FindORB_SLAM_OASIS.cmake
# -----------
# Finds the ORB_SLAM_OASIS headers and library
#
# This will define the following variables:
#
#   ORB_SLAM_OASIS_FOUND        -  system has ORB_SLAM_OASIS
#   ORB_SLAM_OASIS_INCLUDE_DIRS -  ORB_SLAM_OASIS include directories
#   ORB_SLAM_OASIS_LIBRARIES    -  libraries needed to use ORB_SLAM_OASIS
#
# and the following imported targets:
#
#   ORB_SLAM_OASIS::ORB_SLAM_OASIS   - The ORB_SLAM_OASIS library
#

include(FindPackageHandleStandardArgs)

set(PACKAGE_NAME ORB_SLAM_OASIS)

find_path(
  ORB_SLAM_OASIS_INCLUDE_DIR
  NAMES
    "System.h"
  PATH_SUFFIXES
    "include/${PACKAGE_NAME}"
)
find_path(
  CAMERA_MODEL_INCLUDE_DIR
  NAMES
    "GeometricCamera.h"
  PATH_SUFFIXES
    "include/${PACKAGE_NAME}/CameraModels"
)

find_library(
  ORB_SLAM_OASIS_LIBRARY
  NAMES
    "ORB_SLAM_OASIS"
  PATH_SUFFIXES
    "${LIBRARY_PATH_PREFIX}/${PACKAGE_NAME}"
)

find_package_handle_standard_args(
  ORB_SLAM_OASIS
  DEFAULT_MSG
    ORB_SLAM_OASIS_INCLUDE_DIR
    ORB_SLAM_OASIS_LIBRARY
    CAMERA_MODEL_INCLUDE_DIR
)

if (ORB_SLAM_OASIS_FOUND)
  set(ORB_SLAM_OASIS_INCLUDE_DIRS
    "${ORB_SLAM_OASIS_INCLUDE_DIR}"
    "${CAMERA_MODEL_INCLUDE_DIR}"
  )
  set(ORB_SLAM_OASIS_LIBRARIES "${ORB_SLAM_OASIS_LIBRARY}")

  if (NOT TARGET ORB_SLAM_OASIS::ORB_SLAM_OASIS)
    add_library(ORB_SLAM_OASIS::ORB_SLAM_OASIS UNKNOWN IMPORTED)

    set_target_properties(ORB_SLAM_OASIS::ORB_SLAM_OASIS PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
        "${ORB_SLAM_OASIS_INCLUDE_DIRS}"
      IMPORTED_LOCATION
        "${ORB_SLAM_OASIS_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(ORB_SLAM_OASIS_INCLUDE_DIR)
mark_as_advanced(ORB_SLAM_OASIS_LIBRARY)
mark_as_advanced(CAMERA_MODEL_INCLUDE_DIR)

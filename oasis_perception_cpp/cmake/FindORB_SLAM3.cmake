#.rst:
# FindORB_SLAM3.cmake
# -----------
# Finds the ORB-SLAM3 headers and library
#
# This will define the following variables:
#
#   ORB_SLAM3_FOUND        -  system has ORB-SLAM3
#   ORB_SLAM3_INCLUDE_DIRS -  ORB-SLAM3 include directories
#   ORB_SLAM3_LIBRARIES    -  libraries needed to use ORB-SLAM3
#
# and the following imported targets:
#
#   ORB_SLAM3::ORB_SLAM3   - The ORB-SLAM3 library
#

include(FindPackageHandleStandardArgs)

set(PACKAGE_NAME ORB_SLAM3)

find_path(
  ORB_SLAM3_INCLUDE_DIR
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
  ORB_SLAM3_LIBRARY
  NAMES
    "ORB_SLAM3"
  PATH_SUFFIXES
    "${LIBRARY_PATH_PREFIX}/${PACKAGE_NAME}"
)

find_package_handle_standard_args(
  ORB_SLAM3
  DEFAULT_MSG
    ORB_SLAM3_INCLUDE_DIR
    ORB_SLAM3_LIBRARY
    CAMERA_MODEL_INCLUDE_DIR
)

if (ORB_SLAM3_FOUND)
  set(ORB_SLAM3_INCLUDE_DIRS
    "${ORB_SLAM3_INCLUDE_DIR}"
    "${CAMERA_MODEL_INCLUDE_DIR}"
  )
  set(ORB_SLAM3_LIBRARIES "${ORB_SLAM3_LIBRARY}")

  if (NOT TARGET ORB_SLAM3::ORB_SLAM3)
    add_library(ORB_SLAM3::ORB_SLAM3 UNKNOWN IMPORTED)

    set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
        "${ORB_SLAM3_INCLUDE_DIRS}"
      IMPORTED_LOCATION
        "${ORB_SLAM3_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR)
mark_as_advanced(ORB_SLAM3_LIBRARY)
mark_as_advanced(CAMERA_MODEL_INCLUDE_DIR)

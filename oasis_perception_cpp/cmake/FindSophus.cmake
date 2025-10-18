#.rst:
# FindSophus.cmake
# -----------
# Finds the Sophus headers
#
# This will define the following variables:
#
#   SOPHUS_FOUND        -  system has Sophus
#   SOPHUS_INCLUDE_DIRS -  Sophus include directories
#

include(FindPackageHandleStandardArgs)

set(PACKAGE_NAME Sophus)

find_path(
  SOPHUS_INCLUDE_DIR
  NAMES
    "sophus/types.hpp"
  PATH_SUFFIXES
    "include/${PACKAGE_NAME}"
)

find_package_handle_standard_args(
  Sophus
  DEFAULT_MSG
    SOPHUS_INCLUDE_DIR
)

if (Sophus_FOUND)
  set(SOPHUS_FOUND ${Sophus_FOUND})
  set(SOPHUS_INCLUDE_DIRS "${SOPHUS_INCLUDE_DIR}")
endif()

mark_as_advanced(SOPHUS_INCLUDE_DIR)

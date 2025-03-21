#.rst:
# FindEigen3.cmake
# -----------
# Finds the Eigen headers and library
#
# This will define the following variables:
#
#   EIGEN3_FOUND        -  system has Eigen
#   EIGEN3_INCLUDE_DIRS -  Eigen include directories
#

include(FindPackageHandleStandardArgs)

find_path(
  EIGEN3_INCLUDE_DIR
  NAMES
    "Eigen"
  PATH_SUFFIXES
    "include/eigen3"
)

find_package_handle_standard_args(
  Eigen3
  DEFAULT_MSG
    EIGEN3_INCLUDE_DIR
)

if (Eigen3_FOUND)
  set(EIGEN3_FOUND ${Eigen3_FOUND})
  set(EIGEN3_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIR}")
endif()

mark_as_advanced(EIGEN3_INCLUDE_DIR)

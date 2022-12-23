#.rst:
# FindDBoW2.cmake
# -----------
# Finds the DBoW2 headers and library
#
# This will define the following variables:
#
#   DBOW2_FOUND        -  system has DBoW2
#   DBOW2_INCLUDE_DIRS -  DBoW2 include directories
#   DBOW2_LIBRARIES    -  libraries needed to use DBoW2
#
# and the following imported targets:
#
#   DBoW2::DBoW2   - The DBoW2 library
#

include(FindPackageHandleStandardArgs)

find_path(
  DBOW2_INCLUDE_DIR
  NAMES
    "DBoW2/FeatureVector.h"
  PATH_SUFFIXES
    "include/Thirdparty/DBoW2"
)

find_library(
  DBOW2_LIBRARY
  NAMES
    "DBoW2"
  PATH_SUFFIXES
    "${LIBRARY_PATH_PREFIX}"
)

find_package_handle_standard_args(
  DBoW2
  DEFAULT_MSG
    DBOW2_INCLUDE_DIR
    DBOW2_LIBRARY
)

if (DBoW2_FOUND)
  set(DBoW2_FOUND ${DBOW2_FOUND})
  set(DBOW2_INCLUDE_DIRS "${DBOW2_INCLUDE_DIR}")
  set(DBOW2_LIBRARIES "${DBOW2_LIBRARY}")

  if (NOT TARGET DBoW2::DBoW2)
    add_library(DBoW2::DBoW2 UNKNOWN IMPORTED)

    set_target_properties(DBoW2::DBoW2 PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
        "${DBOW2_INCLUDE_DIR}"
      IMPORTED_LOCATION
        "${DBOW2_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(DBOW2_INCLUDE_DIR)
mark_as_advanced(DBOW2_LIBRARY)

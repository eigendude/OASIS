#.rst:
# Findbgslibrary.cmake
# -----------
# Finds the bgslibrary headers and library
#
# This will define the following variables:
#
#   BGSLIBRARY_FOUND        -  system has bgslibrary
#   BGSLIBRARY_INCLUDE_DIRS -  bgslibrary include directories
#   BGSLIBRARY_LIBRARIES    -  libraries needed to use bgslibrary
#
# and the following imported targets:
#
#   bgslibrary::bgslibrary   - The bgslibrary library
#

include(FindPackageHandleStandardArgs)

find_path(
  BGSLIBRARY_INCLUDE_DIR
  NAMES "FrameProcessor.h"
  PATH_SUFFIXES "include"
)

find_library(
  BGSLIBRARY_LIBRARY
  NAMES "bgslibrary_core"
  PATH_SUFFIXES "${LIBRARY_PATH_PREFIX}"
)

find_package_handle_standard_args(
  bgslibrary
  DEFAULT_MSG
    BGSLIBRARY_INCLUDE_DIR
    BGSLIBRARY_LIBRARY
)

if (bgslibrary_FOUND)
  set(BGSLIBRARY_FOUND ${bgslibrary_FOUND})
  set(BGSLIBRARY_INCLUDE_DIRS "${BGSLIBRARY_INCLUDE_DIR}")
  set(BGSLIBRARY_LIBRARIES "${BGSLIBRARY_LIBRARY}")

  if (NOT TARGET bgslibrary::bgslibrary)
    add_library(bgslibrary::bgslibrary UNKNOWN IMPORTED)
    set_target_properties(bgslibrary::bgslibrary PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${BGSLIBRARY_INCLUDE_DIR}"
      IMPORTED_LOCATION "${BGSLIBRARY_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(BGSLIBRARY_INCLUDE_DIR)
mark_as_advanced(BGSLIBRARY_LIBRARY)

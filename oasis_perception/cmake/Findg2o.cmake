#.rst:
# Findg2o.cmake
# -----------
# Finds the g2o headers and library
#
# This will define the following variables:
#
#   G2O_FOUND        -  system has g2o
#   G2O_INCLUDE_DIRS -  g2o include directories
#   G2O_LIBRARIES    -  libraries needed to use g2o
#
# and the following imported targets:
#
#   g2o::g2o   - The g2o library
#

include(FindPackageHandleStandardArgs)

find_path(
  G2O_INCLUDE_DIR
  NAMES
    "config.h"
  PATH_SUFFIXES
    "include/Thirdparty/g2o"
)

find_library(
  G2O_LIBRARY
  NAMES
    "g2o"
  PATH_SUFFIXES
    "${LIBRARY_PATH_PREFIX}"
)

find_package_handle_standard_args(
  g2o
  DEFAULT_MSG
    G2O_INCLUDE_DIR
    G2O_LIBRARY
)

if (g2o_FOUND)
  set(g2o_FOUND ${G2O_FOUND})
  set(G2O_INCLUDE_DIRS "${G2O_INCLUDE_DIR}")
  set(G2O_LIBRARIES "${G2O_LIBRARY}")

  if (NOT TARGET g2o::g2o)
    add_library(g2o::g2o UNKNOWN IMPORTED)

    set_target_properties(g2o::g2o PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
        "${G2O_INCLUDE_DIR}"
      IMPORTED_LOCATION
        "${G2O_LIBRARY}"
    )
  endif()
endif()

mark_as_advanced(G2O_INCLUDE_DIR)
mark_as_advanced(G2O_LIBRARY)

#
# FindMediaPipe.cmake
# -------------------
# Finds the MediaPipe headers and core framework library
#
# This will define the following variables:
#
#   MediaPipe_FOUND        - True if MediaPipe was found
#   MediaPipe_INCLUDE_DIRS - Directories to add to your include path
#   MediaPipe_LIBRARIES    - Libraries needed to link against MediaPipe
#
# And it will create the following imported target:
#
#   MediaPipe::MediaPipe   - The library target for MediaPipe
#

include(FindPackageHandleStandardArgs)

#
# glog dependency
#

find_package(glog REQUIRED)

#
# protobuf dependency
#

find_package(Protobuf REQUIRED)

#
# Find MediaPipe
#

find_path(MediaPipe_INCLUDE_DIR
  NAMES
    mediapipe/framework/packet.h
)

find_library(MediaPipe_MONOLITHIC_LIBRARY
  NAMES
    mediapipe_monolithic
)

find_package_handle_standard_args(MediaPipe
  REQUIRED_VARS MediaPipe_INCLUDE_DIR MediaPipe_MONOLITHIC_LIBRARY
)

mark_as_advanced(MediaPipe_INCLUDE_DIR)
mark_as_advanced(MediaPipe_MONOLITHIC_LIBRARY)

if (MediaPipe_FOUND)
  # Directory that holds the library
  get_filename_component(_mp_libdir "${MediaPipe_MONOLITHIC_LIBRARY}" DIRECTORY)

  # Search for your absl_monolithic.so in that same directory
  find_library(Abseil_MONOLITHIC_LIBRARY
    NAMES
      absl_monolithic
    PATHS
      "${_mp_libdir}"
    NO_DEFAULT_PATH
  )
  if (NOT Abseil_MONOLITHIC_LIBRARY)
    message(FATAL_ERROR "Found mediapipe in ${_mp_libdir} but could not locate absl_monolithic.so")
  endif()

  mark_as_advanced(Abseil_MONOLITHIC_LIBRARY)

  # Build up the final list of includes
  set(MediaPipe_INCLUDE_DIRS
    ${MediaPipe_INCLUDE_DIR}
  )

  # Build up the final list of things to link
  set(MediaPipe_LIBRARIES
    glog::glog
    protobuf::libprotobuf
    ${MediaPipe_MONOLITHIC_LIBRARY}
    ${Abseil_MONOLITHIC_LIBRARY}
  )

  # Create an INTERFACE target so users can just do:
  #
  #   target_link_libraries(myexe PRIVATE MediaPipe::MediaPipe)
  #
  add_library(MediaPipe::MediaPipe INTERFACE IMPORTED)

  # Expose include‑paths, link‑libs and rpath
  set_target_properties(MediaPipe::MediaPipe PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MediaPipe_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${MediaPipe_LIBRARIES}"
    INTERFACE_LINK_OPTIONS "-Wl,-rpath,${_mp_libdir}"
  )
endif()

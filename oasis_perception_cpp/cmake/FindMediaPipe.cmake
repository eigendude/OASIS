#
# FindMediaPipe.cmake
# -------------------
# Finds the private MediaPipe backend used by the OASIS facade
#
# This will define the following variables:
#
#   MediaPipe_FOUND        - True if MediaPipe was found
#   MediaPipe_BACKEND_LIBRARY - Private backend library path
#
# And it will create the following imported target:
#
#   MediaPipe::Backend     - Private backend library target
#

include(FindPackageHandleStandardArgs)

#
# Find MediaPipe
#

find_path(MediaPipe_INCLUDE_DIR
  NAMES
    mediapipe/framework/packet.h
)

find_library(MediaPipe_BACKEND_LIBRARY
  NAMES
    oasis_mediapipe_backend
)

find_package_handle_standard_args(MediaPipe
  REQUIRED_VARS
    MediaPipe_INCLUDE_DIR
    MediaPipe_BACKEND_LIBRARY
)

mark_as_advanced(MediaPipe_INCLUDE_DIR)
mark_as_advanced(MediaPipe_BACKEND_LIBRARY)

if(MediaPipe_FOUND)
  get_filename_component(_mp_libdir "${MediaPipe_BACKEND_LIBRARY}" DIRECTORY)

  add_library(MediaPipe::Backend SHARED IMPORTED)
  set_target_properties(MediaPipe::Backend PROPERTIES
    IMPORTED_LOCATION "${MediaPipe_BACKEND_LIBRARY}"
    INTERFACE_LINK_OPTIONS "-Wl,-rpath,${_mp_libdir}"
  )
endif()

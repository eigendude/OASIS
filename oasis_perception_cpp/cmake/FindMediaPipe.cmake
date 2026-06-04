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

find_path(Protobuf_INCLUDE_DIR
  NAMES
    google/protobuf/stubs/common.h
  PATHS
    /usr/include
    /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
  NO_DEFAULT_PATH
)

find_library(Protobuf_LIBRARY
  NAMES
    protobuf
  PATHS
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/lib
    /lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /lib
  NO_DEFAULT_PATH
)

find_program(Protobuf_PROTOC_EXECUTABLE
  NAMES
    protoc
  PATHS
    /usr/bin
  NO_DEFAULT_PATH
)

find_package(Protobuf REQUIRED)

message(STATUS "Protobuf_VERSION=${Protobuf_VERSION}")
message(STATUS "Protobuf_PROTOC_EXECUTABLE=${Protobuf_PROTOC_EXECUTABLE}")
message(STATUS "Protobuf_INCLUDE_DIRS=${Protobuf_INCLUDE_DIRS}")
message(STATUS "Protobuf_LIBRARIES=${Protobuf_LIBRARIES}")

if (NOT Protobuf_VERSION VERSION_EQUAL "3.21.12")
  message(FATAL_ERROR
    "OASIS C++ consumers require Ubuntu/system Protobuf 3.21.12, "
    "but CMake found Protobuf ${Protobuf_VERSION}"
  )
endif()

foreach(_protobuf_path
  ${Protobuf_PROTOC_EXECUTABLE}
  ${Protobuf_INCLUDE_DIRS}
  ${Protobuf_LIBRARIES}
)
  if (_protobuf_path MATCHES "/mediapipe/")
    message(FATAL_ERROR
      "find_package(Protobuf) resolved a MediaPipe-private path: "
      "${_protobuf_path}"
    )
  endif()
endforeach()

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
  REQUIRED_VARS
    MediaPipe_INCLUDE_DIR
    MediaPipe_MONOLITHIC_LIBRARY
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

  set(MediaPipe_PRIVATE_PROTOBUF_INCLUDE_DIR
    "${MediaPipe_INCLUDE_DIR}/mediapipe_protobuf5"
  )

  if (NOT EXISTS "${MediaPipe_PRIVATE_PROTOBUF_INCLUDE_DIR}/google/protobuf/runtime_version.h")
    message(FATAL_ERROR
      "Found MediaPipe in ${MediaPipe_INCLUDE_DIR} but could not locate "
      "private protobuf5 headers in ${MediaPipe_PRIVATE_PROTOBUF_INCLUDE_DIR}"
    )
  endif()

  # Build up the final list of includes
  set(MediaPipe_INCLUDE_DIRS
    ${MediaPipe_INCLUDE_DIR}
    ${MediaPipe_PRIVATE_PROTOBUF_INCLUDE_DIR}
  )

  # Build up the final list of things to link
  set(MediaPipe_LIBRARIES
    glog::glog
    ${MediaPipe_MONOLITHIC_LIBRARY}
    ${Abseil_MONOLITHIC_LIBRARY}
    protobuf::libprotobuf
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

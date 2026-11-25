# Copyright (C) 2026 Garrett Brown
# This file is part of OASIS - https://github.com/eigendude/OASIS
# SPDX-License-Identifier: Apache-2.0
# See the file LICENSE.txt for more information.

# Embed paths so compiler probes and ExternalProject builds retain the toolchain
file(CONFIGURE OUTPUT "${OASIS_KODI_OUTPUT}" CONTENT [=[
include([==[@OASIS_KODI_TOOLCHAIN@]==])

# Kodi restricts library/header searches to its dependency prefix and Apple SDK
# The native ROS/OpenCV prefixes must also be roots for find_library/find_path
list(APPEND CMAKE_FIND_ROOT_PATH [==[@OASIS_KODI_PREFIX_PATH@]==])
list(REMOVE_DUPLICATES CMAKE_FIND_ROOT_PATH)
]=] @ONLY)

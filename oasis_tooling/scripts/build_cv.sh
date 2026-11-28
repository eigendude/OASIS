#!/bin/bash
################################################################################
#
#  Copyright (C) 2025-2026 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0
#  See the file LICENSE.txt for more information.
#
################################################################################

# Enable strict mode
set -o errexit
set -o pipefail
set -o nounset

#
# Environment paths and configuration
#

# Get the absolute path to this script
SCRIPT_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd -P)"

# Import OpenCV environment and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import CMake environment and config
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Directory setup
#

# Create directories
mkdir -p "${OPENCV_INSTALL_DIR}"
mkdir -p "${OPENCV_PYTHON_INSTALL_DIR}"

OPENCV_PYTHON_BIN="${OPENCV_PYTHON_BIN:-python3}"

#
# Prepare OpenCV and OpenCV Contrib at the same release tag
#

OASIS_SOURCE_OUTPUTS=(--exclude=/.cache/ --exclude=__pycache__/)
prepare_git_source "${OPENCV_SOURCE_DIR}" "${OPENCV_URL}" "refs/tags/${OPENCV_VERSION}" \
  "${CONFIG_DIRECTORY}/opencv/0001-GAPI-Implement-RGBA2Gray-and-GBRA2Gray.patch" \
  "${CONFIG_DIRECTORY}/opencv/0002-Support-FFmpeg-9-codec-capabilities.patch"
prepare_git_source "${OPENCV_CONTRIB_SOURCE_DIR}" "${OPENCV_CONTRIB_URL}" "refs/tags/${OPENCV_VERSION}"

#
# Configure OpenCV
#

echo "Installing OpenCV Python build dependencies..."

# Install or validate the shared NumPy installation.
NUMPY_PYTHON_BIN="${OPENCV_PYTHON_BIN}" \
  "${SCRIPT_DIR}/install_numpy.sh"

"${OPENCV_PYTHON_BIN}" -m pip uninstall \
  --yes \
  --break-system-packages \
  opencv-contrib-python \
  opencv-python \
  opencv-python-headless \
  || true

echo "Discovering OpenCV Python configuration..."
opencv_python_config="$(
  "${OPENCV_PYTHON_BIN}" - <<'PY'
from __future__ import annotations

import sys
import sysconfig
from pathlib import Path

import numpy

executable: str = sys.executable
include: str = sysconfig.get_path("include")
platinclude: str = sysconfig.get_path("platinclude")
purelib: str = sysconfig.get_path("purelib")
platlib: str = sysconfig.get_path("platlib")
numpy_include: str = numpy.get_include()

library_name: str | None = sysconfig.get_config_var("LDLIBRARY")
if library_name is None:
    library_name = sysconfig.get_config_var("LIBRARY")

library_dir: str | None = sysconfig.get_config_var("LIBDIR")
if library_dir is None:
    library_dir = sysconfig.get_config_var("LIBPL")

if library_name is None or library_dir is None:
    raise SystemExit("Unable to determine Python library path")

library: Path = Path(library_dir) / library_name
framework_name: str | None = sysconfig.get_config_var("PYTHONFRAMEWORK")
framework_prefix: str | None = sysconfig.get_config_var("PYTHONFRAMEWORKPREFIX")
if not library.exists() and framework_name and framework_prefix:
    library = (
        Path(framework_prefix)
        / f"{framework_name}.framework"
        / "Versions"
        / sysconfig.get_python_version()
        / framework_name
    )
if not library.exists():
    raise SystemExit(f"Python library not found: {library}")

print(f"executable={executable}")
print(f"include={include}")
print(f"platinclude={platinclude}")
print(f"purelib={purelib}")
print(f"platlib={platlib}")
print(f"numpy_include={numpy_include}")
print(f"library={library}")
PY
)"

while IFS='=' read -r opencv_python_key opencv_python_value; do
  echo "${opencv_python_key}=${opencv_python_value}"
  case "${opencv_python_key}" in
    executable) OPENCV_PYTHON_EXECUTABLE="${opencv_python_value}" ;;
    platinclude) OPENCV_PYTHON_INCLUDE_DIR="${opencv_python_value}" ;;
    library) OPENCV_PYTHON_LIBRARY="${opencv_python_value}" ;;
    numpy_include) OPENCV_PYTHON_NUMPY_INCLUDE_DIR="${opencv_python_value}" ;;
  esac
done <<< "${opencv_python_config}"

echo "Configuring OpenCV..."

cmake_args=(
  -S "${OPENCV_SOURCE_DIR}"
  -B "${OPENCV_BUILD_DIR}"
  # Re-detect the Python ABI when configuring an existing build directory
  -UPYTHON3INTERP_FOUND
  -DBUILD_DOCS=OFF
  -DBUILD_EXAMPLES=OFF
  -DBUILD_opencv_apps=OFF
  -DBUILD_opencv_java=OFF
  # RViz's OGRE omits the Bites component required by ovis
  -DBUILD_opencv_ovis=OFF
  -DBUILD_opencv_python2=OFF
  -DBUILD_opencv_python3=ON
  -DBUILD_opencv_sfm=ON
  -DBUILD_opencv_viz=OFF
  -DBUILD_PERF_TESTS=OFF
  -DBUILD_PROTOBUF=OFF # Use system libprotobuf
  -DBUILD_SHARED_LIBS=ON
  -DBUILD_TESTS=OFF
  -DCMAKE_BUILD_PARALLEL_LEVEL="$(getconf _NPROCESSORS_ONLN)"
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="${OPENCV_INSTALL_DIR}"
  -DENABLE_CCACHE=ON
  -DOPENCV_PYTHON3_INSTALL_PATH="${OPENCV_PYTHON_INSTALL_DIR}"
  -DOPENCV_ENABLE_NONFREE=ON
  -DOPENCV_EXTRA_MODULES_PATH="${OPENCV_CONTRIB_SOURCE_DIR}/modules"
  -DOPENCV_GENERATE_PKGCONFIG=ON
  -DPROTOBUF_UPDATE_FILES=ON
  -DPYTHON3_EXECUTABLE="${OPENCV_PYTHON_EXECUTABLE}"
  -DPYTHON3_INCLUDE_DIR="${OPENCV_PYTHON_INCLUDE_DIR}"
  -DPYTHON3_LIBRARY="${OPENCV_PYTHON_LIBRARY}"
  -DPYTHON3_NUMPY_INCLUDE_DIRS="${OPENCV_PYTHON_NUMPY_INCLUDE_DIR}"
  -DPYTHON3_PACKAGES_PATH="${OPENCV_PYTHON_INSTALL_DIR}"
  -DWITH_FFMPEG=ON
  -DWITH_GSTREAMER=ON
  -DWITH_GTK=OFF
  -DWITH_OPENCL=ON
  -DWITH_QT=OFF
  -DWITH_V4L=ON
)

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # Stale system-protoc output shadows the bundled compatible headers
  for protobuf_file in "${OPENCV_BUILD_DIR}/modules/dnn/"*.pb.cc \
    "${OPENCV_BUILD_DIR}/modules/dnn/"*.pb.h; do
    if [[ -f "${protobuf_file}" ]]; then
      mkdir -p "${OPENCV_BUILD_DIR}/modules/dnn/system-protobuf"
      mv "${protobuf_file}" "${OPENCV_BUILD_DIR}/modules/dnn/system-protobuf/"
    fi
  done

  # Keep OpenCV's private protobuf runtime isolated from MediaPipe's version
  cmake_args+=(
    -DBUILD_PROTOBUF=ON
    -DPROTOBUF_UPDATE_FILES=OFF
    -DCMAKE_CXX_STANDARD=17
    -DCMAKE_INSTALL_RPATH="${OPENCV_INSTALL_DIR}/lib"
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
    -DCMAKE_OSX_ARCHITECTURES="$(uname -m)"
    -DWITH_AVFOUNDATION=ON
    -DWITH_GSTREAMER=OFF
    -DWITH_V4L=OFF
  )
fi

cmake "${cmake_args[@]}"

#
# Build OpenCV
#

echo "Building OpenCV..."
cmake --build "${OPENCV_BUILD_DIR}" \
  --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-$(getconf _NPROCESSORS_ONLN)}"

#
# Install OpenCV
#

echo "Installing OpenCV..."
cmake --install "${OPENCV_BUILD_DIR}"

echo "Verifying OpenCV Python bindings..."
PYTHONPATH="${OPENCV_PYTHON_INSTALL_DIR}:${PYTHONPATH:-}" \
  "${OPENCV_PYTHON_BIN}" - <<'PY'
from __future__ import annotations

import os
from importlib import metadata
from pathlib import Path

import cv2
import numpy as np
from numpy.typing import NDArray

install_path: Path = Path(os.environ["OPENCV_PYTHON_INSTALL_DIR"]).resolve()
cv2_file: Path = Path(cv2.__file__).resolve()
opencv_wheel_names: tuple[str, ...] = (
    "opencv-contrib-python",
    "opencv-python",
    "opencv-python-headless",
)
opencv_wheel_installed: list[str] = []
opencv_wheel_name: str

for opencv_wheel_name in opencv_wheel_names:
    try:
        metadata.version(opencv_wheel_name)
    except metadata.PackageNotFoundError:
        continue
    opencv_wheel_installed.append(opencv_wheel_name)

print("numpy:", np.__version__)
print("cv2:", cv2.__version__)
print("cv2 file:", cv2_file)

if opencv_wheel_installed:
    raise SystemExit(
        "PyPI OpenCV wheel installed in this Python environment: "
        + ", ".join(opencv_wheel_installed)
    )

if not cv2_file.is_relative_to(install_path):
    raise SystemExit(
        f"cv2 was imported from {cv2_file}, expected under {install_path}"
    )

image: NDArray[np.uint8] = np.zeros((32, 32, 3), dtype=np.uint8)
gray: NDArray[np.uint8] = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print("cv2 smoke:", gray.shape, gray.dtype)
PY

echo "OpenCV installed into ${OPENCV_INSTALL_DIR}"

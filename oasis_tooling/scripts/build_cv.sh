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
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Import OpenCV environment and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import CMake environment and config
source "${SCRIPT_DIR}/env_cmake.sh"

#
# Directory setup
#

# Create directories
mkdir -p "${OPENCV_DOWNLOAD_DIR}"
mkdir -p "${OPENCV_INSTALL_DIR}"
mkdir -p "${OPENCV_PYTHON_INSTALL_DIR}"

OPENCV_PYTHON_BIN="${OPENCV_PYTHON_BIN:-python3}"

#
# Download OpenCV
#

if [ ! -f "${OPENCV_ARCHIVE_PATH}" ]; then
  echo "Downloading OpenCV..."
  wget "${OPENCV_URL}" -O "${OPENCV_ARCHIVE_PATH}"
fi

# Download OpenCV Contrib
if [ ! -f "${OPENCV_CONTRIB_ARCHIVE_PATH}" ]; then
  echo "Downloading OpenCV Contrib..."
  wget "${OPENCV_CONTRIB_URL}" -O "${OPENCV_CONTRIB_ARCHIVE_PATH}"
fi

#
# Extract OpenCV
#

echo "Extracting OpenCV..."
rm -rf "${OPENCV_SOURCE_DIR}"
mkdir -p "${OPENCV_SOURCE_DIR}"
tar -zxf "${OPENCV_ARCHIVE_PATH}" --directory="${OPENCV_EXTRACT_DIR}"

#
# Extract OpenCV Contrib
#

echo "Extracting OpenCV Contrib..."
rm -rf "${OPENCV_CONTRIB_SOURCE_DIR}"
mkdir -p "${OPENCV_CONTRIB_SOURCE_DIR}"
tar -zxf "${OPENCV_CONTRIB_ARCHIVE_PATH}" --directory="${OPENCV_EXTRACT_DIR}"

#
# Patch OpenCV
#

echo "Patching OpenCV..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${OPENCV_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/opencv/0001-GAPI-Implement-RGBA2Gray-and-GBRA2Gray.patch"

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
mapfile -t opencv_python_config < <(
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
)

declare -A opencv_python_paths=()
for opencv_python_value in "${opencv_python_config[@]}"; do
  echo "${opencv_python_value}"
  opencv_python_paths["${opencv_python_value%%=*}"]="${opencv_python_value#*=}"
done

OPENCV_PYTHON_EXECUTABLE="${opencv_python_paths[executable]}"
OPENCV_PYTHON_INCLUDE_DIR="${opencv_python_paths[platinclude]}"
OPENCV_PYTHON_LIBRARY="${opencv_python_paths[library]}"
OPENCV_PYTHON_NUMPY_INCLUDE_DIR="${opencv_python_paths[numpy_include]}"

echo "Configuring OpenCV..."

cmake_args=(
  -S "${OPENCV_SOURCE_DIR}"
  -B "${OPENCV_BUILD_DIR}"
  -DBUILD_DOCS=OFF
  -DBUILD_EXAMPLES=OFF
  -DBUILD_opencv_apps=OFF
  -DBUILD_opencv_java=OFF
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

cmake "${cmake_args[@]}"

#
# Build OpenCV
#

echo "Building OpenCV..."
cmake --build "${OPENCV_BUILD_DIR}" --parallel "$(getconf _NPROCESSORS_ONLN)"

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

install_path: Path = Path(os.environ["OPENCV_PYTHON_INSTALL_DIR"]).resolve()
cv2_file: Path = Path(cv2.__file__).resolve()
opencv_wheel_names: tuple[str, ...] = (
    "opencv-contrib-python",
    "opencv-python",
    "opencv-python-headless",
)
opencv_wheel_installed: list[str] = []

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

image: np.ndarray = np.zeros((32, 32, 3), dtype=np.uint8)
gray: np.ndarray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
print("cv2 smoke:", gray.shape, gray.dtype)
PY

echo "OpenCV installed into ${OPENCV_INSTALL_DIR}"

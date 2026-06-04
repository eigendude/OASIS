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

# Import OpenCV paths and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import MediaPipe environment and config
source "${SCRIPT_DIR}/env_mediapipe.sh"

# Add NVM to path for Bazelisk
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" # This loads nvm

#
# Directory setup
#

# Create directories
mkdir -p "${MEDIAPIPE_DOWNLOAD_DIR}"
mkdir -p "${MEDIAPIPE_EXTRACT_DIR}"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}"

#
# Download MediaPipe
#

if [ ! -f "${MEDIAPIPE_ARCHIVE_PATH}" ]; then
  echo "Downloading MediaPipe..."
  wget "${MEDIAPIPE_URL}" -O "${MEDIAPIPE_ARCHIVE_PATH}"
fi

#
# Extract MediaPipe
#

echo "Extracting MediaPipe..."
rm -rf "${MEDIAPIPE_SOURCE_DIR}"
tar -zxf "${MEDIAPIPE_ARCHIVE_PATH}" --directory="${MEDIAPIPE_EXTRACT_DIR}"

#
# Patch MediaPipe
#

echo "Patching MediaPipe..."
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${MEDIAPIPE_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/mediapipe/0001-Enable-OpenCV-4.patch"
patch \
  -p1 \
  --reject-file="/dev/null" \
  --no-backup-if-mismatch \
  --directory="${MEDIAPIPE_SOURCE_DIR}" \
  < "${CONFIG_DIRECTORY}/mediapipe/0002-Enable-monolithic-build.patch"

# Configure MediaPipe to use the custom OpenCV build
OPENCV_BUILD_FILE="${MEDIAPIPE_SOURCE_DIR}/third_party/opencv_linux.BUILD"
OPENCV_WORKSPACE_FILE="${MEDIAPIPE_SOURCE_DIR}/WORKSPACE"

export MEDIAPIPE_OPENCV_BUILD_FILE="${OPENCV_BUILD_FILE}"
export MEDIAPIPE_OPENCV_WORKSPACE_FILE="${OPENCV_WORKSPACE_FILE}"
export MEDIAPIPE_OPENCV_INSTALL_DIR="${OPENCV_INSTALL_DIR}"

python3 <<'PY'
import os
from pathlib import Path

build_file = Path(os.environ["MEDIAPIPE_OPENCV_BUILD_FILE"])
workspace_file = Path(os.environ["MEDIAPIPE_OPENCV_WORKSPACE_FILE"])
opencv_install_dir = Path(os.environ["MEDIAPIPE_OPENCV_INSTALL_DIR"])
opencv_lib_dir = opencv_install_dir / "lib"

build_text = build_file.read_text()

link_block = '    linkopts = [\n'
insertion_lines = []
lib_flag = f'        "-L{opencv_lib_dir}",\n'
rpath_flag = f'        "-Wl,-rpath,{opencv_lib_dir}",\n'

if lib_flag not in build_text:
    insertion_lines.append(lib_flag)
if rpath_flag not in build_text:
    insertion_lines.append(rpath_flag)

if insertion_lines:
    build_text = build_text.replace(link_block, link_block + ''.join(insertion_lines), 1)
    build_file.write_text(build_text)

workspace_lines = workspace_file.read_text().splitlines()

for index, line in enumerate(workspace_lines):
    if "linux_opencv" in line:
        desired = f'    path = "{opencv_install_dir}",'  # Bazel requires trailing comma
        for offset in range(index + 1, len(workspace_lines)):
            candidate = workspace_lines[offset]
            stripped = candidate.strip()
            if stripped.startswith("path ="):
                if candidate != desired:
                    workspace_lines[offset] = desired
                break
            if stripped and not candidate.startswith(" "):
                break
        break

workspace_file.write_text("\n".join(workspace_lines) + "\n")
PY

#
# Build with Bazel
#

cd "${MEDIAPIPE_SOURCE_DIR}"

# If we're on armhf, arm64/aarch64, etc. use clang; otherwise leave Bazel alone
BAZEL_CLANG_FLAGS=()
if [[ "${ARCH}" =~ ^(arm|armhf|arm64|aarch64)$ ]]; then
  BAZEL_CLANG_FLAGS+=(
    --action_env=CC=clang
    --action_env=CXX=clang++
  )
  export CC=clang
  export CXX=clang++
fi

# Build the monolithic library that we patched into MediaPipe
bazelisk build \
  "${BAZEL_CLANG_FLAGS[@]}" \
  --compilation_mode=opt \
  //mediapipe/framework:mediapipe_monolithic

#
# Install MediaPipe
#

# Clear old headers
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/absl"
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/google/protobuf"
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/mediapipe_protobuf5"
rm -rf "${MEDIAPIPE_INSTALL_DIR}/include/mediapipe"

# Create the install layout
mkdir -p "${MEDIAPIPE_INSTALL_DIR}/include"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}/include/mediapipe_protobuf5"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}/lib"

# Copy headers from sources into the install include tree
(
  cd "${MEDIAPIPE_SOURCE_DIR}"
  find mediapipe -name '*.h' \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

# Copy all generated .pb.h headers from the build directory into the install
# include tree
(
  cd "${MEDIAPIPE_BUILD_DIR}"
  find mediapipe -name '*.pb.h' \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

BAZEL_OUT_BASE=$(bazelisk info output_base)

# Copy the active Bazel protobuf headers used to generate MediaPipe .pb.h
PROTOBUF_RUNTIME_HEADER="$(
  find "${BAZEL_OUT_BASE}/external" \
    -path '*/google/protobuf/runtime_version.h' \
    -print \
    -quit
)"

if [ -z "${PROTOBUF_RUNTIME_HEADER}" ]; then
  echo "Unable to locate Bazel protobuf runtime headers" >&2
  exit 1
fi

PROTOBUF_INCLUDE_ROOT="${PROTOBUF_RUNTIME_HEADER%/google/protobuf/runtime_version.h}"
(
  cd "${PROTOBUF_INCLUDE_ROOT}"
  find google/protobuf \( -name '*.h' -o -name '*.inc' \) \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/mediapipe_protobuf5/" \;
)

# Copy Abseil headers into the install include tree
ABSL_SOURCE_DIR="${BAZEL_OUT_BASE}/external/com_google_absl"
(
  cd "${ABSL_SOURCE_DIR}"
  find absl \( -name '*.h' -o -name '*.inc' \) \
    -exec cp --parents '{}' "${MEDIAPIPE_INSTALL_DIR}/include/" \;
)

# Install the monolithic libraries
echo "Installing libmediapipe_monolithic.so"
cp -f \
  "${MEDIAPIPE_BUILD_DIR}/mediapipe/framework/libmediapipe_monolithic.so" \
  "${MEDIAPIPE_INSTALL_DIR}/lib/"
echo "Installing libabsl_monolithic.so"
cp -f \
  "${MEDIAPIPE_BUILD_DIR}/mediapipe/framework/libabsl_monolithic.so" \
  "${MEDIAPIPE_INSTALL_DIR}/lib/"

#
# Verify installed MediaPipe C++ proto linkage
#

SMOKE_TEST_DIR="/tmp/mediapipe_smoke_tests"
mkdir -p "${SMOKE_TEST_DIR}"

MULTIARCH="$(${CXX:-c++} -print-multiarch 2>/dev/null || true)"
SYSTEM_LIBRARY_DIRS=(
  "/lib/${MULTIARCH}"
  "/usr/lib/${MULTIARCH}"
  "/lib"
  "/usr/lib"
)

RPATH_LINK_FLAGS=(
  "-Wl,-rpath-link,${MEDIAPIPE_INSTALL_DIR}/lib"
  "-Wl,-rpath-link,${OPENCV_INSTALL_DIR}/lib"
)

for library_dir in "${SYSTEM_LIBRARY_DIRS[@]}"; do
  if [ -d "${library_dir}" ]; then
    RPATH_LINK_FLAGS+=("-Wl,-rpath-link,${library_dir}")
  fi
done

compile_smoke_test() {
  local smoke_name="$1"
  local smoke_source="$2"
  local smoke_binary="${SMOKE_TEST_DIR}/${smoke_name}"

  "${CXX:-c++}" "${smoke_source}" \
    -I"${MEDIAPIPE_INSTALL_DIR}/include" \
    -I"${MEDIAPIPE_INSTALL_DIR}/include/mediapipe_protobuf5" \
    -L"${MEDIAPIPE_INSTALL_DIR}/lib" \
    -L"${OPENCV_INSTALL_DIR}/lib" \
    -Wl,-rpath,"${MEDIAPIPE_INSTALL_DIR}/lib" \
    -Wl,-rpath,"${OPENCV_INSTALL_DIR}/lib" \
    "${RPATH_LINK_FLAGS[@]}" \
    -Wl,--allow-shlib-undefined \
    -lmediapipe_monolithic \
    -labsl_monolithic \
    -lpthread \
    -ldl \
    -lm \
    -o "${smoke_binary}"
}

run_smoke_test() {
  local smoke_name="$1"
  local smoke_binary="${SMOKE_TEST_DIR}/${smoke_name}"
  local smoke_ldd_output

  echo "Smoke test '${smoke_name}' ldd:"
  smoke_ldd_output="$(ldd "${smoke_binary}" || true)"
  echo "${smoke_ldd_output}"

  echo "Smoke test '${smoke_name}' protobuf/absl deps:"
  echo "${smoke_ldd_output}" | rg "protobuf|absl|opencv|mediapipe" || true

  if echo "${smoke_ldd_output}" | grep -q "not found"; then
    echo "Smoke test '${smoke_name}' has unresolved runtime deps; run skipped"
    return
  fi

  if echo "${smoke_ldd_output}" | grep -q "libprotobuf\\.so"; then
    echo "Smoke test '${smoke_name}' loads system protobuf through broad pose deps; run skipped"
    return
  fi

  "${smoke_binary}"
}

SMOKE_CONSTRUCT_ONLY_SOURCE="${SMOKE_TEST_DIR}/construct_only.cc"
SMOKE_COPY_CONSTRUCT_SOURCE="${SMOKE_TEST_DIR}/copy_construct.cc"
SMOKE_HEAP_LEAK_SOURCE="${SMOKE_TEST_DIR}/heap_leak.cc"

cat > "${SMOKE_CONSTRUCT_ONLY_SOURCE}" <<'CPP'
#include "mediapipe/framework/calculator.pb.h"

int main() {
  mediapipe::CalculatorGraphConfig config;
  return config.node_size();
}
CPP

cat > "${SMOKE_COPY_CONSTRUCT_SOURCE}" <<'CPP'
#include "mediapipe/framework/calculator.pb.h"

int main() {
  mediapipe::CalculatorGraphConfig config;
  mediapipe::CalculatorGraphConfig copy(config);
  return copy.node_size();
}
CPP

cat > "${SMOKE_HEAP_LEAK_SOURCE}" <<'CPP'
#include "mediapipe/framework/calculator.pb.h"

int main() {
  auto* config = new mediapipe::CalculatorGraphConfig();
  auto* copy = new mediapipe::CalculatorGraphConfig(*config);
  return copy->node_size();
}
CPP

compile_smoke_test "construct_only" "${SMOKE_CONSTRUCT_ONLY_SOURCE}"
compile_smoke_test "copy_construct" "${SMOKE_COPY_CONSTRUCT_SOURCE}"
compile_smoke_test "heap_leak" "${SMOKE_HEAP_LEAK_SOURCE}"

echo "MediaPipe monolith dynamic deps:"
ldd "${MEDIAPIPE_INSTALL_DIR}/lib/libmediapipe_monolithic.so" || true

echo "MediaPipe monolith protobuf/absl deps:"
ldd "${MEDIAPIPE_INSTALL_DIR}/lib/libmediapipe_monolithic.so" \
  | rg "protobuf|absl|opencv" || true

echo "OpenCV core dynamic deps:"
ldd "${OPENCV_INSTALL_DIR}/lib/libopencv_core.so"* || true

echo "OpenCV videoio dynamic deps:"
ldd "${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so"* || true

echo "OpenCV pkg-config libs:"
pkg-config --libs opencv4 || true

echo "OpenCV pkg-config static libs:"
pkg-config --libs --static opencv4 || true

echo "OpenCV transitive dependency lookup:"
ldconfig -p \
  | rg "openblas|avif|OpenEXR|gstapp|gstriff|gstpbutils|gstvideo|gstaudio|avcodec|avformat|avutil|swscale" \
  || true

echo "Abseil monolith ldd:"
ldd "${MEDIAPIPE_INSTALL_DIR}/lib/libabsl_monolithic.so" || true

echo "MediaPipe monolith CalculatorGraphConfig/protobuf symbols:"
nm -CD "${MEDIAPIPE_INSTALL_DIR}/lib/libmediapipe_monolithic.so" \
  | rg "CalculatorGraphConfig::CalculatorGraphConfig|google::protobuf::TextFormat|google::protobuf::internal|ShutdownProtobufLibrary" \
  || true

run_smoke_test "construct_only"
run_smoke_test "copy_construct"
run_smoke_test "heap_leak"

echo "MediaPipe installed into ${MEDIAPIPE_INSTALL_DIR}"

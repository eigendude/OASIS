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

MEDIAPIPE_VERBOSE="${MEDIAPIPE_VERBOSE:-0}"
MEDIAPIPE_WHEELHOUSE="${MEDIAPIPE_WHEELHOUSE:-${MEDIAPIPE_INSTALL_DIR}/wheels}"
MEDIAPIPE_PYTHON_BIN="${MEDIAPIPE_PYTHON_BIN:-python3}"
MEDIAPIPE_PYTHON_BIN="$(command -v "${MEDIAPIPE_PYTHON_BIN}")"
MEDIAPIPE_ENABLE_GPU="${MEDIAPIPE_ENABLE_GPU:-1}"

#
# Directory setup
#

# Create directories
mkdir -p "${MEDIAPIPE_DOWNLOAD_DIR}"
mkdir -p "${MEDIAPIPE_EXTRACT_DIR}"
mkdir -p "${MEDIAPIPE_INSTALL_DIR}"
mkdir -p "${MEDIAPIPE_WHEELHOUSE}"

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
  export MEDIAPIPE_USE_CLANG_ACTION_ENV=1
fi

# Keep MediaPipe C++ and Python wheel targets on one Bazel configuration. The
# wheel setup.py build is patched below to use the same analysis-relevant
# options so packaging should mostly copy already-built outputs.
MEDIAPIPE_BAZEL_FLAGS=(
  "${BAZEL_CLANG_FLAGS[@]}"
  --compilation_mode=opt
  --action_env=PYTHON_BIN_PATH="${MEDIAPIPE_PYTHON_BIN}"
  --copt=-DNDEBUG
  --define=ENABLE_ODML_CONVERTER=0
)

if [[ "${MEDIAPIPE_ENABLE_GPU}" != "1" ]]; then
  MEDIAPIPE_BAZEL_FLAGS+=(--define=MEDIAPIPE_DISABLE_GPU=1)
fi

printf -v MEDIAPIPE_BAZEL_FLAGS_SHELL '%q ' "${MEDIAPIPE_BAZEL_FLAGS[@]}"
export MEDIAPIPE_BAZEL_FLAGS_SHELL

MEDIAPIPE_PREBUILD_TARGETS=(
  //mediapipe/framework:mediapipe_monolithic
  //mediapipe/tasks/metadata:image_segmenter_metadata_schema_py
  //mediapipe/tasks/metadata:metadata_schema_py
  //mediapipe/tasks/metadata:object_detector_metadata_schema_py
  //mediapipe/tasks/metadata:schema_py
  //mediapipe/tasks/c:libmediapipe.so
)

echo "MEDIAPIPE_ENABLE_GPU=${MEDIAPIPE_ENABLE_GPU}"
printf 'Bazel flags:\n'
printf '  %q\n' "${MEDIAPIPE_BAZEL_FLAGS[@]}"
printf 'Prebuild Bazel command:\n  bazelisk build'
printf ' %q' "${MEDIAPIPE_BAZEL_FLAGS[@]}" "${MEDIAPIPE_PREBUILD_TARGETS[@]}"
printf '\n'

# Prebuild the monolithic library and Python wheel native targets together
bazelisk build \
  "${MEDIAPIPE_BAZEL_FLAGS[@]}" \
  "${MEDIAPIPE_PREBUILD_TARGETS[@]}"

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

show_ldd() {
  local label="$1"
  shift
  local ldd_output

  echo "${label}:"
  ldd_output="$(ldd "$@" || true)"

  if [ "${MEDIAPIPE_VERBOSE}" = "1" ]; then
    echo "${ldd_output}"
  else
    echo "${ldd_output}" | grep -E "protobuf|absl|opencv|mediapipe|not found" || true
  fi
}

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

  smoke_ldd_output="$(ldd "${smoke_binary}" || true)"

  echo "Smoke test '${smoke_name}' protobuf/absl deps:"
  echo "${smoke_ldd_output}" | grep -E "protobuf|absl|opencv|mediapipe|not found" || true

  if grep -q "not found" <<<"${smoke_ldd_output}"; then
    echo "Smoke test '${smoke_name}' has unresolved runtime deps; run skipped"
    return
  fi

  if grep -q "libprotobuf\\.so" <<<"${smoke_ldd_output}"; then
    # The broad monolith can load OpenCV DNN, which loads system
    # protobuf/absl. Running MediaPipe Protobuf5 object destructors in that
    # mixed process can corrupt teardown, so the smoke tests stop at
    # compile/link. The full OASIS build verifies external linkage.
    echo "Smoke test '${smoke_name}' loads system protobuf through OpenCV DNN"
    echo "Runtime execution skipped to avoid mixed Protobuf5/system teardown"
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

show_ldd \
  "MediaPipe monolith deps" \
  "${MEDIAPIPE_INSTALL_DIR}/lib/libmediapipe_monolithic.so"

show_ldd "OpenCV core deps" "${OPENCV_INSTALL_DIR}/lib/libopencv_core.so"*

show_ldd "OpenCV videoio deps" "${OPENCV_INSTALL_DIR}/lib/libopencv_videoio.so"*

echo "OpenCV pkg-config libs:"
pkg-config --libs opencv4 || true

echo "OpenCV pkg-config static libs:"
pkg-config --libs --static opencv4 || true

echo "OpenCV transitive dependency lookup:"
ldconfig -p \
  | grep -E "openblas|avif|OpenEXR|gstapp|gstriff|gstpbutils|gstvideo|gstaudio|avcodec|avformat|avutil|swscale" \
  || true

show_ldd "Abseil monolith deps" "${MEDIAPIPE_INSTALL_DIR}/lib/libabsl_monolithic.so"

echo "MediaPipe monolith CalculatorGraphConfig/protobuf symbols:"
nm -CD "${MEDIAPIPE_INSTALL_DIR}/lib/libmediapipe_monolithic.so" \
  | grep -E "CalculatorGraphConfig::CalculatorGraphConfig|TextFormat::ParseFromString|ShutdownProtobufLibrary" \
  || true

run_smoke_test "construct_only"
run_smoke_test "copy_construct"
run_smoke_test "heap_leak"

#
# Build and install the MediaPipe Python package
#

echo "Building MediaPipe Python wheel"
cd "${MEDIAPIPE_SOURCE_DIR}"

if [ ! -f setup.py ]; then
  echo "ERROR: MediaPipe setup.py not found; inspect Python build flow" >&2
  exit 1
fi

# The upstream source archive keeps setup.py at version "dev". Stamp the wheel
# with the same MediaPipe version used for the C++ monolith.
export MEDIAPIPE_PYTHON_VERSION="${MEDIAPIPE_VERSION}"
export MEDIAPIPE_ENABLE_GPU="${MEDIAPIPE_ENABLE_GPU}"
export MEDIAPIPE_SKIP_SETUP_BAZEL_BUILD=1
"${MEDIAPIPE_PYTHON_BIN}" <<'PY'
import os
from pathlib import Path

setup_py: Path = Path("setup.py")
version: str = os.environ["MEDIAPIPE_PYTHON_VERSION"]
text: str = setup_py.read_text()
text = text.replace("__version__ = 'dev'", f"__version__ = '{version}'", 1)
text = text.replace(
    "'--define=ENABLE_ODML_CONVERTER=1'",
    "'--define=ENABLE_ODML_CONVERTER=0'",
    1,
)
text = text.replace("self.link_opencv = False", "self.link_opencv = True")
text = text.replace(
    "def _normalize_path(path):\n"
    "  return path.replace('\\\\', '/') if IS_WINDOWS else path\n",
    (
        "def _normalize_path(path):\n"
        "  return path.replace('\\\\', '/') if IS_WINDOWS else path\n"
        "\n"
        "BAZEL_BUILD_OPTIONS = shlex.split(\n"
        "    os.environ['MEDIAPIPE_BAZEL_FLAGS_SHELL']\n"
        ")\n"
    ),
    1,
)
text = text.replace(
    "      bazel_command = [\n"
    "          'bazel',\n"
    "          'build',\n"
    "          '--compilation_mode=opt',\n"
    "          '--action_env=PYTHON_BIN_PATH=' + _normalize_path(sys.executable),\n"
    "          '//mediapipe/tasks/metadata:' + target,\n"
    "      ] + GPU_OPTIONS\n",
    "      bazel_command = [\n"
    "          'bazel',\n"
    "          'build',\n"
    "      ] + BAZEL_BUILD_OPTIONS + [\n"
    "          '//mediapipe/tasks/metadata:' + target,\n"
    "      ]\n",
    1,
)
text = text.replace(
    "    bazel_command = [\n"
    "        'bazel',\n"
    "        'build',\n"
    "        '--compilation_mode=opt',\n"
    "        '--copt=-DNDEBUG',\n"
    "        '--keep_going',\n"
    "        '--define=ENABLE_ODML_CONVERTER=0',\n"
    "        str(ext.bazel_target),\n"
    "    ] + GPU_OPTIONS\n",
    "    bazel_command = [\n"
    "        'bazel',\n"
    "        'build',\n"
    "    ] + BAZEL_BUILD_OPTIONS + [\n"
    "        str(ext.bazel_target),\n"
    "    ]\n",
    1,
)
text = text.replace(
    "      _invoke_shell_command(bazel_command)\n"
    "      _copy_to_build_lib_dir(\n",
    (
        "      print('Setup.py Bazel command:', shlex.join(bazel_command))\n"
        "      if os.environ.get('MEDIAPIPE_SKIP_SETUP_BAZEL_BUILD') != '1':\n"
        "        _invoke_shell_command(bazel_command)\n"
        "      else:\n"
        "        print('Skipping setup.py Bazel build; using prebuilt outputs')\n"
        "      _copy_to_build_lib_dir(\n"
    ),
    1,
)
text = text.replace(
    "    _invoke_shell_command(bazel_command)\n"
    "\n"
    "    ext_bazel_bin_path = os.path.join(\n",
    (
        "    print('Setup.py Bazel command:', shlex.join(bazel_command))\n"
        "    if os.environ.get('MEDIAPIPE_SKIP_SETUP_BAZEL_BUILD') != '1':\n"
        "      _invoke_shell_command(bazel_command)\n"
        "    else:\n"
        "      print('Skipping setup.py Bazel build; using prebuilt outputs')\n"
        "\n"
        "    ext_bazel_bin_path = os.path.join(\n"
    ),
    1,
)
setup_py.write_text(text)
PY

BAZEL_WRAPPER_DIR=""
if ! command -v bazel >/dev/null 2>&1; then
  BAZELISK_BIN="$(command -v bazelisk || true)"
  if [ -z "${BAZELISK_BIN}" ]; then
    echo "ERROR: bazel is missing and bazelisk is unavailable" >&2
    exit 1
  fi

  BAZEL_WRAPPER_DIR="$(mktemp -d)"
  ln -s "${BAZELISK_BIN}" "${BAZEL_WRAPPER_DIR}/bazel"
  export PATH="${BAZEL_WRAPPER_DIR}:${PATH}"
fi

echo "setup.py Bazel build steps will be skipped; using prebuilt outputs"
echo "CI cache check: no Bazel option churn is expected after prebuild"

rm -rf dist
"${MEDIAPIPE_PYTHON_BIN}" setup.py bdist_wheel
rm -f "${MEDIAPIPE_WHEELHOUSE}"/mediapipe-*.whl
cp -f "${MEDIAPIPE_SOURCE_DIR}"/dist/mediapipe-*.whl "${MEDIAPIPE_WHEELHOUSE}/"

MEDIAPIPE_WHEEL="$(
  find "${MEDIAPIPE_WHEELHOUSE}" \
    -maxdepth 1 \
    -name "mediapipe-${MEDIAPIPE_VERSION}-*.whl" \
    -print \
    -quit
)"

if [ -z "${MEDIAPIPE_WHEEL}" ]; then
  echo "ERROR: MediaPipe ${MEDIAPIPE_VERSION} wheel was not produced" >&2
  exit 1
fi

echo "Verifying OASIS OpenCV Python bindings before MediaPipe install"
PYTHONPATH="${OPENCV_PYTHON_INSTALL_DIR}:${PYTHONPATH:-}" \
  "${MEDIAPIPE_PYTHON_BIN}" - <<'PY'
from __future__ import annotations

from importlib import metadata
import os
from pathlib import Path

import cv2

cv2_file: Path = Path(cv2.__file__).resolve()
opencv_python_install_path: Path = Path(
    os.environ["OPENCV_PYTHON_INSTALL_DIR"]
).resolve()
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

print("using cv2:", cv2.__version__, cv2_file)

if opencv_wheel_installed:
    raise SystemExit(
        "PyPI OpenCV wheel installed in this Python environment: "
        + ", ".join(opencv_wheel_installed)
    )

if "opencv_python" in str(cv2_file):
    raise SystemExit(f"cv2 path looks like a PyPI OpenCV wheel: {cv2_file}")

if not cv2_file.is_relative_to(opencv_python_install_path):
    raise SystemExit(
        f"cv2 was imported from {cv2_file}, "
        f"expected under {opencv_python_install_path}"
    )
PY

MEDIAPIPE_REQUIREMENTS_FILE="${MEDIAPIPE_WHEELHOUSE}/mediapipe-runtime-requirements.txt"

echo "Resolving MediaPipe Python runtime requirements"
export MEDIAPIPE_WHEEL
export MEDIAPIPE_REQUIREMENTS_FILE
"${MEDIAPIPE_PYTHON_BIN}" - <<'PY'
from __future__ import annotations

import email
import os
import re
import zipfile
from pathlib import Path

wheel_path: Path = Path(os.environ["MEDIAPIPE_WHEEL"])
requirements_path: Path = Path(os.environ["MEDIAPIPE_REQUIREMENTS_FILE"])
blocked_names: set[str] = {
    "numpy",
    "opencv-contrib-python",
    "opencv-python",
    "opencv-python-headless",
}
requirements: list[str] = []
skipped: list[str] = []

with zipfile.ZipFile(wheel_path) as wheel:
    metadata_name: str | None = next(
        (
            name
            for name in wheel.namelist()
            if name.endswith(".dist-info/METADATA")
        ),
        None,
    )
    if metadata_name is None:
        raise SystemExit(f"Wheel metadata not found: {wheel_path}")

    metadata_text: str = wheel.read(metadata_name).decode("utf-8")

message: email.message.Message = email.message_from_string(metadata_text)

for requirement in message.get_all("Requires-Dist") or []:
    match: re.Match[str] | None = re.match(r"\s*([A-Za-z0-9_.-]+)", requirement)
    if match is None:
        continue

    normalized_name: str = match.group(1).replace("_", "-").lower()
    if normalized_name in blocked_names:
        skipped.append(requirement)
        continue

    requirements.append(requirement)

requirements_path.write_text("\n".join(requirements) + "\n")

for skipped_requirement in skipped:
    print(f"Skipping local OASIS-owned requirement: {skipped_requirement}")

print(f"Wrote {len(requirements)} requirements to {requirements_path}")
PY

echo "Installing MediaPipe Python runtime dependencies"
"${MEDIAPIPE_PYTHON_BIN}" -m pip install \
  --upgrade \
  --break-system-packages \
  numpy
"${MEDIAPIPE_PYTHON_BIN}" -m pip install \
  --upgrade \
  --break-system-packages \
  -r "${MEDIAPIPE_REQUIREMENTS_FILE}"

echo "Installing MediaPipe Python wheel ${MEDIAPIPE_WHEEL}"
"${MEDIAPIPE_PYTHON_BIN}" -m pip install \
  --force-reinstall \
  --break-system-packages \
  --no-deps \
  "${MEDIAPIPE_WHEEL}"

cd "${SCRIPT_DIR}"

"${MEDIAPIPE_PYTHON_BIN}" - <<'PY'
import mediapipe as mp

print("mediapipe:", mp.__version__)
print("file:", mp.__file__)
PY

"${MEDIAPIPE_PYTHON_BIN}" - <<'PY'
import platform

import mediapipe as mp

print("machine:", platform.machine())
print("mediapipe:", mp.__version__)
print("file:", mp.__file__)
print("has solutions:", hasattr(mp, "solutions"))
print("has tasks:", hasattr(mp, "tasks"))
print("has Image:", hasattr(mp, "Image"))
print("has ImageFormat:", hasattr(mp, "ImageFormat"))

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

print("BaseOptions:", python.BaseOptions)
print("PoseLandmarker:", vision.PoseLandmarker)
print("PoseLandmarkerOptions:", vision.PoseLandmarkerOptions)
print("RunningMode:", vision.RunningMode)
print("PoseLandmarkerResult:", vision.PoseLandmarkerResult)

try:
    from mediapipe.tasks.python.vision.pose_landmarker import (
        PoseLandmarksConnections,
    )

    print("PoseLandmarksConnections:", PoseLandmarksConnections)
    print("num connections:", len(PoseLandmarksConnections.POSE_LANDMARKS))
except Exception as e:
    print("PoseLandmarksConnections import failed:", repr(e))
PY

if [ -n "${BAZEL_WRAPPER_DIR}" ]; then
  rm -rf "${BAZEL_WRAPPER_DIR}"
fi

echo "MediaPipe installed into ${MEDIAPIPE_INSTALL_DIR}"

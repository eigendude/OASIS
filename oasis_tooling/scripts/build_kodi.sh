#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2026 Garrett Brown
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

# Import CMake paths and config
source "${SCRIPT_DIR}/env_cmake.sh"

# Import OpenCV paths and config
source "${SCRIPT_DIR}/env_cv.sh"

# Import Kodi paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

#
# Load OASIS environment
#

set +o nounset
source "${OASIS_INSTALL_DIRECTORY}/setup.bash"
set -o nounset

KODI_BUILD_JOBS="${KODI_BUILD_JOBS:-${CMAKE_BUILD_PARALLEL_LEVEL:-$(getconf _NPROCESSORS_ONLN)}}"

mkdir -p "${KODI_DOWNLOAD_DIR}"
KODI_DOWNLOAD_DIR="$(cd "${KODI_DOWNLOAD_DIR}" && pwd -P)"

# Publish each download only after wget completes
download_if_missing() (
  local url="$1" file="$2" temp
  if [[ ! -f "${file}" ]]; then
    temp="$(mktemp "${file}.XXXXXXXX")"
    trap 'rm -f -- "${temp}"' EXIT
    wget "${url}" -O "${temp}"
    mv "${temp}" "${file}"
  fi
)

patches=(
  0001-depends-Remove-git-dependency.patch
  0002-depends-Build-selected-addons-in-parallel.patch
)
if [[ "${OSTYPE}" != "darwin"* ]]; then
  patches+=(
    0001-temp-Wait-until-Wayland-display-is-available.patch
    0002-temp-Wayland-Disable-window-decorations.patch
  )
  if [[ -f "${CONFIG_DIRECTORY}/kodi/0003-temp-Force-${HOSTNAME}-dimensions.patch" ]]; then
    patches+=("0003-temp-Force-${HOSTNAME}-dimensions.patch")
  fi
fi

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # Serialize changes to the checkout, build products, and installed bundle
  exec 9>"${KODI_DIRECTORY}/build.lock"
  lockf -s 9

  export JAVA_HOME="$(brew --prefix openjdk)/libexec/openjdk.jdk/Contents/Home"
  export PATH="$(brew --prefix m4)/bin:${JAVA_HOME}/bin:${PATH}"
  depends_args=(
    --host="${KODI_DEPENDS_HOST}"
    --with-platform=macos
    --enable-debug=no
    --with-tarballs="${KODI_DOWNLOAD_DIR}"
  )

  # Keep pins locally so returning to a previously built revision works offline
  kodi_repo="${KODI_DOWNLOAD_DIR}/kodi.git"
  git init --bare --quiet "${kodi_repo}"
  if ! git -C "${kodi_repo}" cat-file -e "${KODI_VERSION}^{commit}" 2>/dev/null; then
    git -C "${kodi_repo}" fetch --depth=1 --no-tags "${KODI_URL}.git" \
      "${KODI_VERSION}:refs/pins/${KODI_VERSION}"
  fi

  # Kodi's top-level depends stamps do not track changes to package recipes.
  # Give incompatible inputs their own checkout, CMake build and install prefix.
  # TexturePacker also compiles these Kodi files outside tools/depends.
  workspace_key="$(
    {
      # Bash 3 clears errexit in command substitutions
      set -e
      git -C "${kodi_repo}" ls-tree "${KODI_VERSION}" -- tools/depends \
        cmake/addons xbmc/addons/kodi-dev-kit xbmc/guilib/XBTF.cpp \
        xbmc/guilib/XBTF.h xbmc/guilib/XBTFReader.h xbmc/guilib/TextureFormats.h \
        xbmc/utils/EndianSwap.h
      for patch_name in "${patches[@]}"; do
        git hash-object "${CONFIG_DIRECTORY}/kodi/${patch_name}"
      done
      printf '%s\n' "${depends_args[@]}" "${JAVA_HOME}" "${MACOSX_DEPLOYMENT_TARGET:-}"
      for flag in CC CXX CFLAGS CPPFLAGS CXXFLAGS LDFLAGS SDKROOT; do
        if [[ -n "${!flag:-}" ]]; then
          printf '%s=%s\n' "${flag}" "${!flag}"
        fi
      done
      uname -r
      xcodebuild -version
      xcrun --sdk macosx --show-sdk-path
      xcrun --sdk macosx --show-sdk-build-version
      xcrun --sdk macosx --find clang
      command -v ccache || echo 'ccache unavailable'
    } | git hash-object --stdin
  )"
  kodi_workspace="${KODI_DIRECTORY}/work/${workspace_key}"
  KODI_SOURCE_DIR="${kodi_workspace}/src"
  KODI_BUILD_DIR="${kodi_workspace}/build"
  KODI_DEPENDS_DIR="${kodi_workspace}/depends"

  if [[ ! -e "${KODI_SOURCE_DIR}/.git" ]]; then
    git -C "${kodi_repo}" worktree add --force --detach "${KODI_SOURCE_DIR}" "${KODI_VERSION}"
  else
    # Checkout updates only changed files, preserving mtimes and in-tree stamps
    git -C "${KODI_SOURCE_DIR}" checkout --quiet --detach "${KODI_VERSION}"
  fi
else
  prepare_git_source "${KODI_SOURCE_DIR}" \
    "${KODI_URL}.git" "${KODI_VERSION}"
fi

# Use physical paths even when the workspace is reached through a symlink
mkdir -p "${KODI_BUILD_DIR}" "${KODI_DEPENDS_DIR}"
KODI_SOURCE_DIR="$(cd "${KODI_SOURCE_DIR}" && pwd -P)"
KODI_BUILD_DIR="$(cd "${KODI_BUILD_DIR}" && pwd -P)"
KODI_DEPENDS_DIR="$(cd "${KODI_DEPENDS_DIR}" && pwd -P)"
KODI_DEPENDS_SRC="${KODI_SOURCE_DIR}/tools/depends"
cd "${KODI_BUILD_DIR}"

for patch_name in "${patches[@]}"; do
  patch_path="${CONFIG_DIRECTORY}/kodi/${patch_name}"
  if patch -p1 --dry-run --forward --directory="${KODI_SOURCE_DIR}" \
    < "${patch_path}" > /dev/null 2>&1; then
    patch -p1 --forward --directory="${KODI_SOURCE_DIR}" < "${patch_path}"
  elif ! patch -p1 --dry-run --reverse --directory="${KODI_SOURCE_DIR}" \
    < "${patch_path}" > /dev/null 2>&1; then
    echo "Kodi patch does not match the pinned source: ${patch_name}" >&2
    exit 1
  fi
done

#
# Configure Kodi
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  echo "Building Kodi's macOS dependencies..."
  (
    cd "${KODI_DEPENDS_SRC}"
    if [[ ! -f .oasis-configured || ! -f Makefile.include ]]; then
      ./bootstrap
      ./configure "${depends_args[@]}" --prefix="${KODI_DEPENDS_DIR}"
      touch .oasis-configured
    fi
    awk -F= '$1 == "USE_CCACHE" {print "Kodi depends ccache: " $2}' Makefile.include
    make -j"${KODI_BUILD_JOBS}"
  )

  # Configure determines the dependency prefix from the selected SDK and CPU
  KODI_TARGET_PREFIX="$(awk -F= '$1 == "PREFIX" {print $2}' \
    "${KODI_DEPENDS_SRC}/Makefile.include")"
  KODI_NATIVE_PREFIX="$(awk -F= '$1 == "NATIVEPREFIX" {print $2}' \
    "${KODI_DEPENDS_SRC}/Makefile.include")"
  KODI_CMAKE="${KODI_NATIVE_PREFIX}/bin/cmake"
  KODI_TOOLCHAIN="${KODI_TARGET_PREFIX}/share/Toolchain.cmake"
  test -x "${KODI_CMAKE}"
  test -f "${KODI_TOOLCHAIN}"
  KODI_PREFIX_PATH="${CMAKE_PREFIX_PATH//:/;}"
  KODI_APP_TOOLCHAIN="${KODI_BUILD_DIR}/Toolchain-oasis.cmake"
  "${KODI_CMAKE}" \
    -DOASIS_KODI_TOOLCHAIN="${KODI_TOOLCHAIN}" \
    -DOASIS_KODI_PREFIX_PATH="${KODI_PREFIX_PATH}" \
    -DOASIS_KODI_OUTPUT="${KODI_APP_TOOLCHAIN}" \
    -P "${CONFIG_DIRECTORY}/kodi/GenerateToolchain-macos.cmake"
  KODI_RUNTIME_PATH=""
  IFS=: read -r -a KODI_EXTERNAL_PREFIXES <<< "${CMAKE_PREFIX_PATH}"
  for prefix in "${KODI_EXTERNAL_PREFIXES[@]}"; do
    KODI_RUNTIME_PATH+="${KODI_RUNTIME_PATH:+;}${prefix}/lib"
  done

  cmake_args=(
    -G "Unix Makefiles"
    -DCMAKE_TOOLCHAIN_FILE="${KODI_APP_TOOLCHAIN}"
    -DCMAKE_DEPENDS_USE_COMPILER=OFF
    -DCMAKE_PREFIX_PATH="${KODI_PREFIX_PATH}"
    -DCMAKE_BUILD_RPATH="${KODI_RUNTIME_PATH}"
    -DCMAKE_INSTALL_RPATH="${KODI_RUNTIME_PATH}"
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON
    -DCORE_PLATFORM_NAME=osx
    -DENABLE_INTERNAL_CROSSGUID=ON
  )
else
  KODI_CMAKE=cmake
  cmake_args=(
    -DAPP_RENDER_SYSTEM="${APP_RENDER_SYSTEM}"
    -DCMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}"
    -DCORE_PLATFORM_NAME=wayland
    -DENABLE_CEC=OFF
    -DENABLE_INTERNAL_FFMPEG=ON
    -DENABLE_INTERNAL_SWIG=ON
  )
fi

(
  if [[ "${OSTYPE}" != "darwin"* ]]; then
    export PATH="${CMAKE_BIN_DIRECTORY}:${PATH}"
    export PKG_CONFIG_PATH="${KODI_DEPENDS_DIR}/${KODI_DEPENDS_TARGET}/lib/pkgconfig"
  fi
  echo "Configuring Kodi..."
  "${KODI_CMAKE}" -S "${KODI_SOURCE_DIR}" -B "${KODI_BUILD_DIR}" \
    "${cmake_args[@]}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${KODI_INSTALL_DIR}" \
    -DENABLE_CEC=OFF \
    -DENABLE_LLD="${ENABLE_LLD}" \
    -DENABLE_ROS2=ON \
    -DENABLE_TESTING=OFF \
    -DTARBALL_DIR="${KODI_DOWNLOAD_DIR}" \
    -DOpenCV_DIR="${OpenCV_DIR}" \
    -DOpenCV_ROOT="${OpenCV_ROOT}"
)

#
# Build and install Kodi and its add-ons
#

if [[ "${OSTYPE}" == "darwin"* ]]; then
  # Stage add-ons before building the app so they are included in its resources
  addon_dir="${KODI_BUILD_DIR}/addons"
  addon_args=(INSTALL_PREFIX="${addon_dir}")
else
  echo "Building Kodi..."
  make -C "${KODI_BUILD_DIR}" -j"${KODI_BUILD_JOBS}"
  echo "Installing Kodi..."
  make -C "${KODI_BUILD_DIR}" install

  addon_dir="${KODI_INSTALL_DIR}/share/kodi/addons"
  addon_args=(PREFIX="${KODI_INSTALL_DIR}" EXTRA_CMAKE_ARGS="-DAPP_RENDER_SYSTEM=gl")
fi

echo "Building Kodi add-ons..."
addons=(peripheral.joystick screensaver.matrixtrails)
# The launchers also apply to CMake projects configured by ExternalProject
CMAKE_C_COMPILER_LAUNCHER=ccache CMAKE_CXX_COMPILER_LAUNCHER=ccache \
  make -C "${KODI_DEPENDS_SRC}/target/binary-addons" -j"${KODI_BUILD_JOBS}" \
    ADDONS="${addons[*]}" ADDON_BUILD_TARGET=build-addons \
    "${addon_args[@]}" \
    TARBALLS_LOCATION="${KODI_DOWNLOAD_DIR}"

for addon in "${addons[@]}"; do
  if [[ ! -f "${addon_dir}/${addon}/addon.xml" ]]; then
    echo "Missing Kodi add-on manifest: ${addon_dir}/${addon}/addon.xml" >&2
    exit 1
  fi
done

WEATHER_ICONS_ADDON="resource.images.weathericons.hd.animated"
WEATHER_ICONS_URL="https://mirrors.kodi.tv/addons/piers/${WEATHER_ICONS_ADDON}/${WEATHER_ICONS_ADDON}-1.0.0.zip"
WEATHER_ICONS_ZIP="${KODI_DOWNLOAD_DIR}/${WEATHER_ICONS_ADDON}-1.0.0.zip"
download_if_missing "${WEATHER_ICONS_URL}" "${WEATHER_ICONS_ZIP}"
if [[ ! -f "${addon_dir}/${WEATHER_ICONS_ADDON}/addon.xml" ]]; then
  mkdir -p "${addon_dir}"
  unzip -o "${WEATHER_ICONS_ZIP}" -d "${addon_dir}"
fi

if [[ "${OSTYPE}" == "darwin"* ]]; then
  echo "Building Kodi.app..."
  "${KODI_CMAKE}" --build "${KODI_BUILD_DIR}" \
    --parallel "${KODI_BUILD_JOBS}"

  # Install the app bundle with its staged resources
  test -x "${KODI_BUILD_DIR}/Kodi.app/Contents/MacOS/Kodi"
  mkdir -p "${KODI_INSTALL_DIR}"
  ditto "${KODI_BUILD_DIR}/Kodi.app" "${KODI_INSTALL_DIR}/Kodi.app"

  # Bundle assembly changes load commands and invalidates the linker signatures
  codesign --force --deep --sign - --timestamp=none \
    "${KODI_INSTALL_DIR}/Kodi.app"
  codesign --verify --deep --strict "${KODI_INSTALL_DIR}/Kodi.app"
  echo "Installed ${KODI_INSTALL_DIR}/Kodi.app"
fi

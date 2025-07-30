#!/bin/bash
################################################################################
#
#  Copyright (C) 2022-2025 Garrett Brown
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

# Import OASIS depends paths and config
source "${SCRIPT_DIR}/env_kodi.sh"

#
# Install Kodi dependencies
#

if [[ "${OSTYPE}" != "darwin"* ]]; then
  sudo apt install -y \
    autoconf \
    automake \
    autopoint \
    autotools-dev \
    ccache \
    cmake \
    curl \
    debhelper \
    default-jre \
    doxygen \
    gawk \
    gcc \
    gdc \
    gettext \
    gperf \
    libasound2-dev \
    libass-dev \
    libavahi-client-dev \
    libavahi-common-dev \
    libbluetooth-dev \
    libbluray-dev \
    libbz2-dev \
    libcdio-dev \
    libcec-dev \
    libcrossguid-dev \
    libcurl4-openssl-dev \
    libcwiid-dev \
    libdbus-1-dev \
    libdisplay-info-dev \
    libdrm-dev \
    libegl1-mesa-dev \
    libenca-dev \
    libexiv2-dev \
    libflac-dev \
    libflatbuffers-dev \
    libfmt-dev \
    libfontconfig-dev \
    libfreetype6-dev \
    libfribidi-dev \
    libfstrcmp-dev \
    libgcrypt-dev \
    libgif-dev \
    libgl1-mesa-dev \
    libgles2-mesa-dev \
    libglu1-mesa-dev \
    libgnutls28-dev \
    libgpg-error-dev \
    libgtest-dev \
    libinput-dev \
    libiso9660-dev \
    libjpeg-dev \
    liblcms2-dev \
    liblirc-dev \
    libltdl-dev \
    liblzo2-dev \
    libmicrohttpd-dev \
    libmysqlclient-dev \
    libnfs-dev \
    libogg-dev \
    libp8-platform-dev \
    libpcre3-dev \
    libplist-dev \
    libpng-dev \
    libpulse-dev \
    libshairplay-dev \
    libsmbclient-dev \
    libspdlog-dev \
    libsqlite3-dev \
    libssl-dev \
    libtag1-dev \
    libtiff5-dev \
    libtinyxml-dev \
    libtinyxml2-dev \
    libtool \
    libudev-dev \
    libunistring-dev \
    libva-dev \
    libvdpau-dev \
    libvorbis-dev \
    libxmu-dev \
    libxrandr-dev \
    libxslt1-dev \
    libxt-dev \
    lsb-release \
    meson \
    nasm \
    ninja-build \
    python3-dev \
    python3-pil \
    python3-pip \
    rapidjson-dev \
    swig \
    unzip \
    uuid-dev \
    zip \
    zlib1g-dev \

  if [ "${ENABLE_LLD}" = "ON" ]; then
    sudo apt install -y lld
  fi

  # Wayland dependencies
  sudo apt install -y \
    libglew-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    waylandpp-dev
fi

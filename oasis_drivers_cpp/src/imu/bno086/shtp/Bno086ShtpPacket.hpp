/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace OASIS::IMU::BNO086
{
/*!\brief Decoded SHTP packet after the 4-byte transport header */
struct Bno086ShtpPacket
{
  /*!
   * \brief SHTP channel carried in header byte 2
   *
   * Units: channel ID in range [0, 5]
   */
  std::uint8_t channel{0};

  /*!
   * \brief SHTP sequence number carried in header byte 3
   *
   * Units: wrapping packet counter
   */
  std::uint8_t sequence{0};

  /*!
   * \brief True when the packet length field has the continuation bit set
   */
  bool continuation{false};

  /*!
   * \brief Payload bytes after the 4-byte SHTP header
   *
   * Units: raw bytes, possibly empty
   */
  std::vector<std::uint8_t> payload;
};
} // namespace OASIS::IMU::BNO086

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace OASIS::IMU::BNO086
{
constexpr std::size_t kShtpHeaderBytes = 4;
constexpr std::size_t kMaxShtpPacketBytes = 1024;
constexpr std::size_t kShtpChannelCount = 6;

constexpr std::uint8_t kShtpChannelCommand = 0;
constexpr std::uint8_t kShtpChannelControl = 2;
constexpr std::uint8_t kShtpChannelReports = 3;
constexpr std::uint8_t kShtpChannelWakeReports = 4;
constexpr std::uint8_t kShtpChannelGyroRotationVector = 5;

/*!\brief Parsed fields from a BNO086 SHTP packet header */
struct Bno086ShtpHeader
{
  /*!
   * \brief Packet length with the continuation bit masked off
   *
   * Units: bytes, including the 4-byte SHTP header
   */
  std::uint16_t length{0};

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
};

bool ParseBno086ShtpHeader(const std::uint8_t* header_bytes, Bno086ShtpHeader& header);
bool IsSaneBno086ShtpChannel(std::uint8_t channel);
bool LooksLikeBno086ShtpHeaderPrefix(const std::uint8_t* header_bytes);
} // namespace OASIS::IMU::BNO086

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/shtp/Bno086ShtpHeader.hpp"

namespace OASIS::IMU::BNO086
{
bool ParseBno086ShtpHeader(const std::uint8_t* header_bytes, Bno086ShtpHeader& header)
{
  const std::uint16_t lengthField = static_cast<std::uint16_t>(
      header_bytes[0] | (static_cast<std::uint16_t>(header_bytes[1]) << 8));

  header.continuation = (lengthField & 0x8000U) != 0U;
  header.length = static_cast<std::uint16_t>(lengthField & 0x7FFFU);
  header.channel = header_bytes[2];
  header.sequence = header_bytes[3];

  if (header.length == 0)
    return true;

  if (header.length < kShtpHeaderBytes || header.length > kMaxShtpPacketBytes)
    return false;

  if (!IsSaneBno086ShtpChannel(header.channel))
    return false;

  return true;
}

bool IsSaneBno086ShtpChannel(std::uint8_t channel)
{
  return channel < kShtpChannelCount;
}

bool LooksLikeBno086ShtpHeaderPrefix(const std::uint8_t* header_bytes)
{
  const std::uint16_t lengthField = static_cast<std::uint16_t>(
      header_bytes[0] | (static_cast<std::uint16_t>(header_bytes[1]) << 8));
  const std::size_t length = static_cast<std::size_t>(lengthField & 0x7FFFU);
  return length >= kShtpHeaderBytes && length <= kMaxShtpPacketBytes;
}
} // namespace OASIS::IMU::BNO086

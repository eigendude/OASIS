/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086Transport.hpp"

#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

using namespace OASIS::IMU::BNO086;

namespace
{
class FdTransport : public Bno086Transport
{
public:
  explicit FdTransport(int fd) : Bno086Transport(fd) {}
};
} // namespace

TEST(Bno086Transport, readPacketProbesHeaderThenRereadsFullRawPacket)
{
  std::array<int, 2> pipeFds{-1, -1};
  ASSERT_EQ(::pipe(pipeFds.data()), 0);

  FdTransport transport{pipeFds[0]};

  const std::vector<std::uint8_t> headerProbe{0x07, 0x00, 0x03, 0x2A};
  const std::vector<std::uint8_t> fullPacket{0x07, 0x00, 0x03, 0x2A, 0xAA, 0xBB, 0xCC};

  ASSERT_EQ(::write(pipeFds[1], headerProbe.data(), headerProbe.size()),
            static_cast<ssize_t>(headerProbe.size()));
  ASSERT_EQ(::write(pipeFds[1], fullPacket.data(), fullPacket.size()),
            static_cast<ssize_t>(fullPacket.size()));
  ::close(pipeFds[1]);

  Bno086ShtpPacket packet;
  ASSERT_TRUE(transport.ReadPacket(packet, 1));

  EXPECT_EQ(packet.channel, 0x03);
  EXPECT_EQ(packet.sequence, 0x2A);
  EXPECT_FALSE(packet.continuation);
  ASSERT_EQ(packet.payload.size(), 3U);
  EXPECT_EQ(packet.payload[0], 0xAA);
  EXPECT_EQ(packet.payload[1], 0xBB);
  EXPECT_EQ(packet.payload[2], 0xCC);
}

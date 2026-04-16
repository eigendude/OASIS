/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace OASIS::IMU::BNO086
{
struct Bno086TransportConfig
{
  std::string i2c_device{"/dev/i2c-1"};
  std::uint8_t i2c_address{0x4B};
};

struct Bno086ShtpPacket
{
  std::uint8_t channel{0};
  std::uint8_t sequence{0};
  bool continuation{false};
  std::vector<std::uint8_t> payload;
};

class Bno086Transport
{
public:
  Bno086Transport() = default;
  ~Bno086Transport();

  bool Open(const Bno086TransportConfig& config);
  void Close();

  bool IsOpen() const;

  bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload);
  bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms);

private:
  struct ShtpHeader
  {
    std::uint16_t length{0};
    std::uint8_t channel{0};
    std::uint8_t sequence{0};
    bool continuation{false};
  };

  bool ReadTransaction(std::uint8_t* buffer,
                       std::size_t size,
                       const std::chrono::steady_clock::time_point& deadline) const;
  bool WriteExact(const std::uint8_t* buffer, std::size_t size) const;
  bool ParseHeader(const std::uint8_t* header_bytes, ShtpHeader& header) const;
  bool IsSaneChannel(std::uint8_t channel) const;
  bool ValidateFullPacket(const std::vector<std::uint8_t>& raw_packet, ShtpHeader& header) const;
  bool LooksLikePseudoPayload(const Bno086ShtpPacket& packet) const;
  bool LooksLikeShtpHeader(const std::uint8_t* header_bytes) const;

  Bno086TransportConfig m_config{};
  int m_fd{-1};
  std::array<std::uint8_t, 6> m_txSequence{};
};
} // namespace OASIS::IMU::BNO086

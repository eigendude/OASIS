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
  std::uint16_t raw_length{0};
  std::uint16_t packet_length{0};
  std::uint8_t channel{0};
  std::uint8_t sequence{0};
  bool continuation{false};
  std::vector<std::uint8_t> payload;
};

/*!
 * \brief Runtime diagnostics for low-level SHTP transport reads
 *
 * Durations are measured around Bno086Transport::ReadPacket. Counters are
 * cumulative since the transport was opened.
 */
struct Bno086TransportDiagnostics
{
  //! Latest wall-clock duration of one ReadPacket call, in ms
  double latest_transport_read_duration_ms{0.0};

  //! Maximum wall-clock duration observed for ReadPacket, in ms
  double max_transport_read_duration_ms{0.0};

  //! ReadPacket calls whose duration exceeded requested timeout plus slop
  std::uint64_t transport_read_over_timeout_count{0};

  //! Number of ReadPacket calls made through this transport
  std::uint64_t transport_read_calls{0};
};

class Bno086Transport
{
public:
  Bno086Transport() = default;
  virtual ~Bno086Transport();

  virtual bool Open(const Bno086TransportConfig& config);
  void Close();

  bool IsOpen() const;

  const Bno086TransportDiagnostics& GetDiagnostics() const;

  virtual bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload);
  virtual bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms);
  static bool ParseShtpHeaderBytes(const std::array<std::uint8_t, 4>& header_bytes,
                                   Bno086ShtpPacket& packet_header);

private:
  struct ShtpHeader
  {
    std::uint16_t raw_length{0};
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
  static bool IsSaneChannel(std::uint8_t channel);
  bool ValidateFullPacket(const std::vector<std::uint8_t>& raw_packet, ShtpHeader& header) const;
  bool LooksLikePseudoPayload(const Bno086ShtpPacket& packet) const;
  bool LooksLikeShtpHeader(const std::uint8_t* header_bytes) const;

  Bno086TransportConfig m_config{};
  int m_fd{-1};
  std::array<std::uint8_t, 6> m_txSequence{};
  Bno086TransportDiagnostics m_diagnostics{};
};
} // namespace OASIS::IMU::BNO086

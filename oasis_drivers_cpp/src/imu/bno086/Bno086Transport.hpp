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

#include <sys/types.h>

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

  //! Number of 4-byte SHTP header ReadTransaction calls
  std::uint64_t transport_header_read_calls{0};

  //! Number of full raw-packet ReadTransaction calls
  std::uint64_t transport_full_packet_read_calls{0};

  //! Latest 4-byte header ReadTransaction duration, in ms
  double latest_header_read_duration_ms{0.0};

  //! Maximum 4-byte header ReadTransaction duration, in ms
  double max_header_read_duration_ms{0.0};

  //! Latest full raw-packet ReadTransaction duration, in ms
  double latest_full_packet_read_duration_ms{0.0};

  //! Maximum full raw-packet ReadTransaction duration, in ms
  double max_full_packet_read_duration_ms{0.0};

  //! Header ReadTransaction calls that exceeded remaining timeout plus slop
  std::uint64_t header_read_over_timeout_count{0};

  //! Full raw-packet ReadTransaction calls that exceeded timeout plus slop
  std::uint64_t full_packet_read_over_timeout_count{0};

  //! Bytes requested by the latest full raw-packet read
  std::uint32_t latest_full_packet_read_bytes{0};

  //! Largest byte count requested by any full raw-packet read
  std::uint32_t max_full_packet_read_bytes{0};

  //! Total bytes requested by full raw-packet reads
  std::uint64_t full_packet_read_bytes_total{0};

  //! Number of outer ReadPacket parse/read attempts
  std::uint64_t read_packet_attempts{0};

  //! ReadPacket attempts that saw a zero-length header
  std::uint64_t read_packet_zero_length_headers{0};

  //! ReadPacket attempts that saw an invalid probe header
  std::uint64_t read_packet_invalid_headers{0};

  //! ReadPacket attempts that saw an invalid full packet
  std::uint64_t read_packet_invalid_full_packets{0};

  //! ReadPacket attempts that discarded a pseudo-payload packet
  std::uint64_t read_packet_pseudo_payloads{0};

  //! ReadPacket exits because the deadline expired before header read
  std::uint64_t read_packet_deadline_expired_before_header{0};

  //! ReadPacket exits because the deadline expired before full packet read
  std::uint64_t read_packet_deadline_expired_before_full_packet{0};

  //! ReadPacket exits because the deadline expired after a pseudo payload
  std::uint64_t read_packet_deadline_expired_after_pseudo_payload{0};

  //! Full raw-packet reads started with less than the low-budget threshold
  std::uint64_t full_packet_read_started_with_low_budget{0};

  //! Requested byte count for the latest low-level read call
  std::uint32_t latest_transaction_requested_bytes{0};

  //! Actual byte count returned by the latest low-level read call
  std::uint32_t latest_transaction_bytes{0};

  //! Latest low-level read call duration, in ms
  double latest_transaction_duration_ms{0.0};

  //! Maximum low-level read call duration observed, in ms
  double max_transaction_duration_ms{0.0};

  //! Low-level read calls that exceeded remaining timeout plus slop
  std::uint64_t transaction_over_timeout_count{0};

  //! Low-level read calls that failed or returned a short read
  std::uint64_t transaction_failures{0};
};

class Bno086Transport
{
public:
  Bno086Transport() = default;
  virtual ~Bno086Transport();

  virtual bool Open(const Bno086TransportConfig& config);
  void Close();

  virtual bool IsOpen() const;

  const Bno086TransportDiagnostics& GetDiagnostics() const;

  virtual bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload);
  virtual bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms);
  static bool ParseShtpHeaderBytes(const std::array<std::uint8_t, 4>& header_bytes,
                                   Bno086ShtpPacket& packet_header);

private:
  enum class ReadTransactionKind
  {
    Header,
    FullPacket,
  };

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
                       const std::chrono::steady_clock::time_point& deadline,
                       ReadTransactionKind kind);
  virtual ssize_t ReadFromDevice(std::uint8_t* buffer, std::size_t size) const;
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

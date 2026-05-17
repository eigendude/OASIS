/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "imu/bno086/shtp/Bno086ShtpHeader.hpp"
#include "imu/bno086/shtp/Bno086ShtpPacket.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace OASIS::IMU::BNO086
{
struct Bno086TransportConfig
{
  /*!
   * \brief Linux I2C device used for BNO086 SHTP transactions
   *
   * Units: filesystem path
   */
  std::string i2c_device;

  /*!
   * \brief 7-bit BNO086 I2C address passed to the Linux I2C driver
   *
   * Units: address in range [0x00, 0x7F]
   */
  std::uint8_t i2c_address{0};
};

struct Bno086TransportStats
{
  /*!
   * \brief Number of I2C read transactions that failed before timeout
   *
   * Units: read transaction failures
   */
  std::uint64_t i2c_read_error_count{0};

  /*!
   * \brief Number of I2C read transactions that expired without data
   *
   * Units: read transaction timeouts
   */
  std::uint64_t i2c_read_timeout_count{0};

  /*!
   * \brief Number of invalid SHTP header probes observed
   *
   * Units: header probes rejected by SHTP header validation
   */
  std::uint64_t invalid_header_count{0};

  /*!
   * \brief Number of all-zero SHTP header probes observed
   *
   * Units: header probes where all four bytes were zero
   */
  std::uint64_t all_zero_header_count{0};

  /*!
   * \brief Number of invalid full packet reads observed
   *
   * Units: full SHTP packets rejected after reading the probed length
   */
  std::uint64_t invalid_full_packet_count{0};
};

class Bno086Transport
{
public:
  Bno086Transport() = default;
  virtual ~Bno086Transport();

  bool Open(const Bno086TransportConfig& config);
  void Close();

  bool IsOpen() const;

  virtual bool WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload);
  virtual bool ReadPacket(Bno086ShtpPacket& packet, int timeout_ms);
  Bno086TransportStats GetStats() const;

protected:
  explicit Bno086Transport(int fd);

private:
  bool ReadTransaction(std::uint8_t* buffer,
                       std::size_t size,
                       const std::chrono::steady_clock::time_point& deadline) const;
  bool WriteExact(const std::uint8_t* buffer, std::size_t size) const;
  bool ValidateFullPacket(const std::vector<std::uint8_t>& raw_packet,
                          Bno086ShtpHeader& header) const;
  bool LooksLikePseudoPayload(const Bno086ShtpPacket& packet) const;

  Bno086TransportConfig m_config{};
  int m_fd{-1};
  std::array<std::uint8_t, 6> m_txSequence{};
  mutable Bno086TransportStats m_stats{};
};
} // namespace OASIS::IMU::BNO086

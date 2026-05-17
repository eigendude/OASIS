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
  std::string i2c_device{"/dev/i2c-1"};

  /*!
   * \brief 7-bit BNO086 I2C address passed to the Linux I2C driver
   *
   * Units: address in range [0x00, 0x7F]
   */
  std::uint8_t i2c_address{0x4B};
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
};
} // namespace OASIS::IMU::BNO086

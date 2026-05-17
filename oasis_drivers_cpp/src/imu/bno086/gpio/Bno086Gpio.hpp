/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <chrono>
#include <string>

namespace OASIS::IMU::BNO086
{
struct Bno086GpioConfig
{
  /*!
   * \brief Linux GPIO chip device used for the interrupt line
   *
   * Units: filesystem path
   */
  std::string chip_device;

  /*!
   * \brief GPIO line offset connected to active-low H_INTN
   *
   * Units: GPIO line number, expected range [0, +inf)
   */
  unsigned int line_offset{0};
};

class Bno086Gpio
{
public:
  enum class WaitResult
  {
    Asserted,
    Timeout,
    Error,
  };

  Bno086Gpio() = default;
  ~Bno086Gpio();

  bool Open(const Bno086GpioConfig& config);
  void Close();

  bool IsOpen() const;
  bool IsAssertedLow() const;

  WaitResult WaitForAssertedLow(int timeout_ms,
                                std::chrono::steady_clock::time_point& asserted_at) const;

private:
  Bno086GpioConfig m_config{};
  int m_chipFd{-1};
  int m_lineFd{-1};
};
} // namespace OASIS::IMU::BNO086

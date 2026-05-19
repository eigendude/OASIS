/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <chrono>
#include <cstdint>
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
  /*!
   * \brief Timestamp returned with an asserted-low interrupt wait
   */
  struct AssertedLowTimestamp
  {
    /*!
     * \brief Candidate H_INTN assertion timestamp
     *
     * Units: std::chrono::steady_clock time domain
     */
    std::chrono::steady_clock::time_point asserted_at{};

    /*!
     * \brief True when \ref asserted_at came from a Linux GPIO event
     */
    bool has_gpio_event_timestamp{false};

    /*!
     * \brief Raw Linux GPIO event timestamp
     *
     * Units: nanoseconds in the kernel event timestamp clock basis
     */
    std::uint64_t gpio_event_timestamp_ns{0};
  };

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

  WaitResult WaitForAssertedLow(int timeout_ms, AssertedLowTimestamp& asserted_timestamp) const;

private:
  Bno086GpioConfig m_config{};
  int m_chipFd{-1};
  int m_lineFd{-1};
};
} // namespace OASIS::IMU::BNO086

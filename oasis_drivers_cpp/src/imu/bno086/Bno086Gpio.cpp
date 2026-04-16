/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/Bno086Gpio.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr const char* kConsumerName = "bno086_imu_driver";
}

Bno086Gpio::~Bno086Gpio()
{
  Close();
}

bool Bno086Gpio::Open(const Bno086GpioConfig& config)
{
  Close();

  m_config = config;

  m_chipFd = ::open(m_config.chip_device.c_str(), O_RDONLY | O_CLOEXEC);
  if (m_chipFd < 0)
    return false;

  struct gpio_v2_line_request request;
  std::memset(&request, 0, sizeof(request));

  request.num_lines = 1;
  request.offsets[0] = m_config.line_offset;

  request.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_FALLING;

  std::strncpy(request.consumer, kConsumerName, sizeof(request.consumer) - 1);
  request.consumer[sizeof(request.consumer) - 1] = '\0';

  if (ioctl(m_chipFd, GPIO_V2_GET_LINE_IOCTL, &request) < 0)
  {
    Close();
    return false;
  }

  m_lineFd = request.fd;

  const int flags = fcntl(m_lineFd, F_GETFL, 0);
  if (flags >= 0)
    fcntl(m_lineFd, F_SETFL, flags | O_NONBLOCK);

  return true;
}

void Bno086Gpio::Close()
{
  if (m_lineFd >= 0)
  {
    ::close(m_lineFd);
    m_lineFd = -1;
  }

  if (m_chipFd >= 0)
  {
    ::close(m_chipFd);
    m_chipFd = -1;
  }
}

bool Bno086Gpio::IsOpen() const
{
  return m_lineFd >= 0;
}

bool Bno086Gpio::IsAssertedLow() const
{
  if (!IsOpen())
    return false;

  struct gpio_v2_line_values values;
  std::memset(&values, 0, sizeof(values));
  values.mask = 0x1ULL;

  if (ioctl(m_lineFd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0)
    return false;

  const bool lineIsHigh = (values.bits & 0x1ULL) != 0ULL;
  return !lineIsHigh;
}

Bno086Gpio::WaitResult Bno086Gpio::WaitForAssertedLow(
    int timeout_ms, std::chrono::steady_clock::time_point& asserted_at) const
{
  if (!IsOpen())
    return WaitResult::Error;

  if (IsAssertedLow())
  {
    asserted_at = std::chrono::steady_clock::now();
    return WaitResult::Asserted;
  }

  struct pollfd pollFd;
  std::memset(&pollFd, 0, sizeof(pollFd));
  pollFd.fd = m_lineFd;
  pollFd.events = POLLIN;

  const int pollResult = ::poll(&pollFd, 1, timeout_ms);
  if (pollResult == 0)
    return WaitResult::Timeout;

  if (pollResult < 0)
  {
    if (errno == EINTR)
      return WaitResult::Timeout;

    return WaitResult::Error;
  }

  struct gpio_v2_line_event event;
  while (true)
  {
    const ssize_t bytesRead = ::read(m_lineFd, &event, sizeof(event));
    if (bytesRead == static_cast<ssize_t>(sizeof(event)))
      continue;

    if (bytesRead < 0 && errno == EAGAIN)
      break;

    if (bytesRead <= 0)
      break;
  }

  if (IsAssertedLow())
  {
    asserted_at = std::chrono::steady_clock::now();
    return WaitResult::Asserted;
  }

  return WaitResult::Timeout;
}
} // namespace OASIS::IMU::BNO086

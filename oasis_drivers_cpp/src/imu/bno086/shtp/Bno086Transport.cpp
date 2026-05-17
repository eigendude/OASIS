/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/bno086/shtp/Bno086Transport.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace OASIS::IMU::BNO086
{
namespace
{
constexpr auto kWriteRetryDelay = std::chrono::microseconds(200);
constexpr auto kWriteRetryTimeout = std::chrono::milliseconds(10);
} // namespace

Bno086Transport::~Bno086Transport()
{
  Close();
}

Bno086Transport::Bno086Transport(int fd) : m_fd(fd)
{
}

bool Bno086Transport::Open(const Bno086TransportConfig& config)
{
  Close();

  m_config = config;
  m_stats = Bno086TransportStats{};
  m_fd = ::open(m_config.i2c_device.c_str(), O_RDWR | O_NONBLOCK);
  if (m_fd < 0)
    return false;

  if (ioctl(m_fd, I2C_SLAVE, static_cast<unsigned long>(m_config.i2c_address)) < 0)
  {
    Close();
    return false;
  }

  m_txSequence.fill(0);
  return true;
}

void Bno086Transport::Close()
{
  if (m_fd >= 0)
  {
    ::close(m_fd);
    m_fd = -1;
  }
}

bool Bno086Transport::IsOpen() const
{
  return m_fd >= 0;
}

bool Bno086Transport::WritePacket(std::uint8_t channel, const std::vector<std::uint8_t>& payload)
{
  if (!IsOpen())
    return false;

  if (channel >= m_txSequence.size())
    return false;

  if (payload.size() > (0x7FFFU - kShtpHeaderBytes))
    return false;

  const std::uint16_t packetLength = static_cast<std::uint16_t>(kShtpHeaderBytes + payload.size());
  const std::uint8_t sequence = m_txSequence[channel];

  std::vector<std::uint8_t> buffer(packetLength, 0);
  buffer[0] = static_cast<std::uint8_t>(packetLength & 0xFF);
  buffer[1] = static_cast<std::uint8_t>((packetLength >> 8) & 0x7F);
  buffer[2] = channel;
  buffer[3] = sequence;

  for (std::size_t i = 0; i < payload.size(); ++i)
    buffer[kShtpHeaderBytes + i] = payload[i];

  const bool writeOk = WriteExact(buffer.data(), buffer.size());
  if (writeOk)
    ++m_txSequence[channel];

  return writeOk;
}

bool Bno086Transport::ReadPacket(Bno086ShtpPacket& packet, int timeout_ms)
{
  if (!IsOpen())
    return false;

  const auto now = std::chrono::steady_clock::now();
  const auto deadline = now + std::chrono::milliseconds(std::max(timeout_ms, 0));

  while (true)
  {
    std::array<std::uint8_t, kShtpHeaderBytes> header{};
    if (!ReadTransaction(header.data(), header.size(), deadline))
      return false;

    if (std::all_of(header.begin(), header.end(), [](std::uint8_t value) { return value == 0; }))
    {
      ++m_stats.all_zero_header_count;
      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      continue;
    }

    Bno086ShtpHeader probedHeader;
    if (!ParseBno086ShtpHeader(header.data(), probedHeader))
    {
      ++m_stats.invalid_header_count;
      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      continue;
    }

    if (probedHeader.length == 0)
    {
      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      continue;
    }

    std::vector<std::uint8_t> rawPacket(probedHeader.length, 0);
    if (!ReadTransaction(rawPacket.data(), rawPacket.size(), deadline))
      return false;

    Bno086ShtpHeader packetHeader;
    if (!ValidateFullPacket(rawPacket, packetHeader))
    {
      ++m_stats.invalid_full_packet_count;
      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      continue;
    }

    const std::size_t payloadLength = packetHeader.length - kShtpHeaderBytes;
    packet.channel = packetHeader.channel;
    packet.sequence = packetHeader.sequence;
    packet.continuation = packetHeader.continuation;
    packet.payload.assign(payloadLength, 0);

    if (payloadLength == 0)
      return true;

    std::copy(rawPacket.begin() + static_cast<std::ptrdiff_t>(kShtpHeaderBytes), rawPacket.end(),
              packet.payload.begin());

    if (LooksLikePseudoPayload(packet))
    {
      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      continue;
    }

    return true;
  }
}

Bno086TransportStats Bno086Transport::GetStats() const
{
  return m_stats;
}

bool Bno086Transport::ReadTransaction(std::uint8_t* buffer,
                                      std::size_t size,
                                      const std::chrono::steady_clock::time_point& deadline) const
{
  if (!IsOpen())
    return false;

  while (true)
  {
    const ssize_t bytesRead = ::read(m_fd, buffer, size);
    if (bytesRead == static_cast<ssize_t>(size))
      return true;

    if (bytesRead < 0 && errno == EINTR)
      continue;

    if (bytesRead < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        ++m_stats.i2c_read_timeout_count;
        return false;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }

    if (bytesRead == 0 && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }

    if (bytesRead == 0)
      ++m_stats.i2c_read_timeout_count;
    else
      ++m_stats.i2c_read_error_count;

    return false;
  }
}

bool Bno086Transport::WriteExact(const std::uint8_t* buffer, std::size_t size) const
{
  if (!IsOpen())
    return false;

  const auto deadline = std::chrono::steady_clock::now() + kWriteRetryTimeout;
  std::size_t offset = 0;
  while (offset < size)
  {
    const ssize_t bytesWritten = ::write(m_fd, buffer + offset, size - offset);
    if (bytesWritten > 0)
    {
      offset += static_cast<std::size_t>(bytesWritten);
      continue;
    }

    if (bytesWritten < 0 && errno == EINTR)
      continue;

    if (bytesWritten < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
        std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(kWriteRetryDelay);
      continue;
    }

    return false;
  }

  return true;
}

bool Bno086Transport::ValidateFullPacket(const std::vector<std::uint8_t>& raw_packet,
                                         Bno086ShtpHeader& header) const
{
  if (raw_packet.size() < kShtpHeaderBytes)
    return false;

  if (!ParseBno086ShtpHeader(raw_packet.data(), header))
    return false;

  if (header.length == 0)
    return false;

  if (header.length != raw_packet.size())
    return false;

  if (header.continuation && header.length == kShtpHeaderBytes)
    return false;

  return true;
}

bool Bno086Transport::LooksLikePseudoPayload(const Bno086ShtpPacket& packet) const
{
  if (packet.payload.size() != kShtpHeaderBytes)
    return false;

  if (packet.channel > 2)
    return false;

  return LooksLikeBno086ShtpHeaderPrefix(packet.payload.data());
}
} // namespace OASIS::IMU::BNO086

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "display/ssd1305/Ssd1305Device.hpp"
#include "display/ssd1305/Ssd1305Transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <deque>
#include <exception>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

using OASIS::Display::SSD1305_DISPLAY_WIDTH;
using OASIS::Display::Ssd1305Device;
using OASIS::Display::Ssd1305DeviceConfig;
using OASIS::Display::Ssd1305Framebuffer;
using OASIS::Display::Ssd1305Transport;

namespace
{
class FakeTransport final : public Ssd1305Transport
{
public:
  void Open(const std::string& device, int address) override
  {
    events.push_back("open:" + device + ":" + std::to_string(address));
    open = true;
  }

  void Close() noexcept override
  {
    events.emplace_back("close");
    open = false;
  }

  bool IsOpen() const noexcept override { return open; }

  void Write(std::span<const std::uint8_t> transaction) override
  {
    events.emplace_back("write");
    transactions.emplace_back(transaction.begin(), transaction.end());
    if (failures.empty())
      return;
    const std::exception_ptr failure = failures.front();
    failures.pop_front();
    std::rethrow_exception(failure);
  }

  bool open{false};
  std::vector<std::string> events;
  std::vector<std::vector<std::uint8_t>> transactions;
  std::deque<std::exception_ptr> failures;
};

struct DeviceFixture
{
  DeviceFixture()
  {
    auto transport = std::make_unique<FakeTransport>();
    fake = transport.get();
    device = std::make_unique<Ssd1305Device>(
        Ssd1305DeviceConfig{
            .i2c_device = "/dev/i2c-1",
            .i2c_address = 0x3C,
            .width = OASIS::Display::SSD1305_DISPLAY_WIDTH,
            .height = OASIS::Display::SSD1305_DISPLAY_HEIGHT,
            .column_offset = 4,
            .contrast = 0xFF,
        },
        std::move(transport));
  }

  FakeTransport* fake{nullptr};
  std::unique_ptr<Ssd1305Device> device;
};

bool ContainsSequence(const std::vector<std::uint8_t>& bytes,
                      std::initializer_list<std::uint8_t> sequence)
{
  return std::search(bytes.begin(), bytes.end(), sequence.begin(), sequence.end()) != bytes.end();
}

std::size_t FindTransaction(const std::vector<std::vector<std::uint8_t>>& transactions,
                            const std::vector<std::uint8_t>& expected)
{
  const auto match = std::find(transactions.begin(), transactions.end(), expected);
  return static_cast<std::size_t>(std::distance(transactions.begin(), match));
}

std::size_t CountTransaction(const std::vector<std::vector<std::uint8_t>>& transactions,
                             const std::vector<std::uint8_t>& expected)
{
  return static_cast<std::size_t>(std::count(transactions.begin(), transactions.end(), expected));
}
} // namespace

TEST(Ssd1305Device, InitializationStaysOffAndUsesProduct4567Configuration)
{
  DeviceFixture fixture;
  fixture.device->Initialize();
  ASSERT_EQ(fixture.fake->transactions.size(), 4U);
  const auto& config = fixture.fake->transactions[0];
  const auto& bounds = fixture.fake->transactions[1];
  ASSERT_GE(config.size(), 2U);
  EXPECT_EQ(config[0], 0x00);
  EXPECT_EQ(config[1], 0xAE);
  EXPECT_FALSE(ContainsSequence(config, {0xAF}));
  EXPECT_TRUE(ContainsSequence(config, {0x20, 0x00}));
  EXPECT_TRUE(ContainsSequence(config, {0xA8, 0x1F}));
  EXPECT_TRUE(ContainsSequence(config, {0xD9, 0xD2}));
  EXPECT_TRUE(ContainsSequence(config, {0xAD, 0x8E}));
  EXPECT_TRUE(ContainsSequence(config, {0xD8, 0x05}));
  EXPECT_TRUE(ContainsSequence(config, {0x91, 0x3F, 0x3F, 0x3F, 0x3F}));
  EXPECT_TRUE(ContainsSequence(config, {0xDB, 0x34}));
  EXPECT_TRUE(ContainsSequence(bounds, {0x8D, 0x14}));
  EXPECT_TRUE(ContainsSequence(bounds, {0x21, 0x04, 0x83}));
  EXPECT_TRUE(ContainsSequence(bounds, {0x22, 0x00, 0x03}));
  EXPECT_FALSE(ContainsSequence(bounds, {0xB0}));
  EXPECT_EQ(fixture.fake->transactions[2], (std::vector<std::uint8_t>{0x00, 0xA1}));
  EXPECT_EQ(fixture.fake->transactions[3], (std::vector<std::uint8_t>{0x00, 0xC8}));
  EXPECT_EQ(FindTransaction(fixture.fake->transactions, {0x00, 0xAF}),
            fixture.fake->transactions.size());
  EXPECT_EQ(FindTransaction(fixture.fake->transactions, {0x00, 0xA0}),
            fixture.fake->transactions.size());
  EXPECT_EQ(FindTransaction(fixture.fake->transactions, {0x00, 0xC0}),
            fixture.fake->transactions.size());
}

TEST(Ssd1305Device, RepeatedInitializationProgramsOrientationEveryTime)
{
  DeviceFixture fixture;

  fixture.device->Initialize();
  fixture.device->Initialize();

  EXPECT_EQ(CountTransaction(fixture.fake->transactions, {0x00, 0xA1}), 2U);
  EXPECT_EQ(CountTransaction(fixture.fake->transactions, {0x00, 0xC8}), 2U);
}

TEST(Ssd1305Device, SuccessfulRetryProgramsOrientationAfterWriteFailure)
{
  DeviceFixture fixture;
  fixture.fake->failures.push_back(
      std::make_exception_ptr(std::system_error(EIO, std::generic_category())));

  EXPECT_THROW(fixture.device->Initialize(), std::system_error);
  EXPECT_NO_THROW(fixture.device->Initialize());

  EXPECT_EQ(CountTransaction(fixture.fake->transactions, {0x00, 0xA1}), 1U);
  EXPECT_EQ(CountTransaction(fixture.fake->transactions, {0x00, 0xC8}), 1U);
}

TEST(Ssd1305Device, PageDataUsesBoundedTransactionsWithControlBytes)
{
  DeviceFixture fixture;
  Ssd1305Framebuffer::Buffer framebuffer{};
  for (std::size_t index = 0; index < framebuffer.size(); ++index)
    framebuffer[index] = static_cast<std::uint8_t>(index);

  fixture.device->WritePage(framebuffer, 2);
  ASSERT_EQ(fixture.fake->transactions.size(), 5U);
  EXPECT_EQ(fixture.fake->transactions[0],
            (std::vector<std::uint8_t>{0x00, 0x21, 0x04, 0x83, 0x22, 0x02, 0x02}));
  for (std::size_t index = 1; index < fixture.fake->transactions.size(); ++index)
  {
    EXPECT_EQ(fixture.fake->transactions[index].size(), 33U);
    EXPECT_EQ(fixture.fake->transactions[index][0], 0x40);
  }
}

TEST(Ssd1305Device, ShortWriteFailsOneTransactionWithoutResuming)
{
  DeviceFixture fixture;
  fixture.fake->failures.push_back(
      std::make_exception_ptr(std::runtime_error("short SSD1305 I2C write: expected 30 bytes but "
                                                 "wrote 4")));
  try
  {
    fixture.device->Initialize();
    FAIL() << "Expected short write failure";
  }
  catch (const std::runtime_error& error)
  {
    const std::string message = error.what();
    EXPECT_NE(message.find("write SSD1305 commands to /dev/i2c-1 at address 0x3c"),
              std::string::npos);
    EXPECT_NE(message.find("expected 30 bytes but wrote 4"), std::string::npos);
  }
  EXPECT_EQ(fixture.fake->transactions.size(), 1U);
}

TEST(Ssd1305Device, SystemErrorPropagatesWithoutGlobalErrno)
{
  DeviceFixture fixture;
  fixture.fake->failures.push_back(std::make_exception_ptr(
      std::system_error(EIO, std::generic_category(), "write SSD1305 I2C transaction")));
  try
  {
    fixture.device->Initialize();
    FAIL() << "Expected system error";
  }
  catch (const std::system_error& error)
  {
    EXPECT_EQ(error.code(), std::make_error_code(std::errc::io_error));
    const std::string message = error.what();
    EXPECT_NE(message.find("write SSD1305 commands to /dev/i2c-1 at address 0x3c"),
              std::string::npos);
    EXPECT_NE(message.find("write SSD1305 I2C transaction"), std::string::npos);
  }
  EXPECT_EQ(fixture.fake->transactions.size(), 1U);
}

TEST(Ssd1305Device, RecoveryReopensRestoresFullFrameThenEnables)
{
  DeviceFixture fixture;
  Ssd1305Framebuffer::Buffer framebuffer{};
  framebuffer.fill(0xA5);
  fixture.device->Open();
  fixture.fake->events.clear();
  fixture.fake->transactions.clear();

  fixture.device->Recover(framebuffer, 0x40, true);
  ASSERT_GE(fixture.fake->events.size(), 2U);
  EXPECT_EQ(fixture.fake->events[0], "close");
  EXPECT_EQ(fixture.fake->events[1], "open:/dev/i2c-1:60");

  const auto data_transaction_count =
      std::count_if(fixture.fake->transactions.begin(), fixture.fake->transactions.end(),
                    [](const std::vector<std::uint8_t>& transaction)
                    { return !transaction.empty() && transaction.front() == 0x40; });
  EXPECT_EQ(data_transaction_count, 16);
  ASSERT_FALSE(fixture.fake->transactions.empty());
  EXPECT_EQ(fixture.fake->transactions.back(), (std::vector<std::uint8_t>{0x00, 0xAF}));

  const std::size_t segment_remap = FindTransaction(fixture.fake->transactions, {0x00, 0xA1});
  const std::size_t com_scan = FindTransaction(fixture.fake->transactions, {0x00, 0xC8});
  const std::size_t contrast_index =
      FindTransaction(fixture.fake->transactions, {0x00, 0x81, 0x40});
  const std::size_t first_data_index = static_cast<std::size_t>(std::distance(
      fixture.fake->transactions.begin(),
      std::find_if(fixture.fake->transactions.begin(), fixture.fake->transactions.end(),
                   [](const std::vector<std::uint8_t>& transaction)
                   { return !transaction.empty() && transaction.front() == 0x40; })));
  const std::size_t display_on = FindTransaction(fixture.fake->transactions, {0x00, 0xAF});
  EXPECT_LT(segment_remap, com_scan);
  EXPECT_LT(com_scan, contrast_index);
  EXPECT_LT(contrast_index, first_data_index);
  EXPECT_LT(first_data_index, display_on);

  const auto first_on =
      std::find(fixture.fake->transactions.begin(), fixture.fake->transactions.end(),
                std::vector<std::uint8_t>{0x00, 0xAF});
  EXPECT_EQ(first_on, fixture.fake->transactions.end() - 1);

  const auto contrast =
      std::find(fixture.fake->transactions.begin(), fixture.fake->transactions.end(),
                std::vector<std::uint8_t>{0x00, 0x81, 0x40});
  const auto first_data =
      std::find_if(fixture.fake->transactions.begin(), fixture.fake->transactions.end(),
                   [](const std::vector<std::uint8_t>& transaction)
                   { return !transaction.empty() && transaction.front() == 0x40; });
  EXPECT_LT(contrast, first_data);
}

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "power_meter/LinuxI2cDeviceProbe.hpp"
#include "power_meter/LinuxI2cRegisterDevice.hpp"

#include <cerrno>
#include <system_error>

#include <gtest/gtest.h>

using OASIS::PowerMeter::ACS37800_PROBE_REGISTER;
using OASIS::PowerMeter::ACS37800_REGISTER_WIDTH;
using OASIS::PowerMeter::ClassifyI2cTransferError;
using OASIS::PowerMeter::DecodeAcs37800RegisterBytes;
using OASIS::PowerMeter::I2cProbeStatus;
using OASIS::PowerMeter::I2cRegisterReadTransfer;
using OASIS::PowerMeter::ValidateI2cTransferResult;

TEST(LinuxI2cDeviceProbe, BuildsReadOnlyCombinedMeasurementRegisterRead)
{
  I2cRegisterReadTransfer transfer(0x60, ACS37800_PROBE_REGISTER);

  ASSERT_EQ(transfer.messages.size(), 2U);
  EXPECT_EQ(transfer.register_selector, ACS37800_PROBE_REGISTER);
  EXPECT_EQ(transfer.messages[0].addr, 0x60);
  EXPECT_EQ(transfer.messages[0].flags, 0);
  EXPECT_EQ(transfer.messages[0].len, 1);
  EXPECT_EQ(transfer.messages[0].buf, &transfer.register_selector);

  EXPECT_EQ(transfer.messages[1].addr, 0x60);
  EXPECT_EQ(transfer.messages[1].flags, I2C_M_RD);
  EXPECT_EQ(transfer.messages[1].len, ACS37800_REGISTER_WIDTH);
  EXPECT_EQ(transfer.messages[1].buf, transfer.read_data.data());
}

TEST(LinuxI2cDeviceProbe, DecodesLeastSignificantByteFirst)
{
  EXPECT_EQ(DecodeAcs37800RegisterBytes({0x78, 0x56, 0x34, 0x12}), 0x12345678U);
}

TEST(LinuxI2cDeviceProbe, RejectsShortNackAndTransportFailures)
{
  EXPECT_THROW(ValidateI2cTransferResult(1, 0, "/dev/i2c-1"), std::runtime_error);

  try
  {
    ValidateI2cTransferResult(-1, EREMOTEIO, "/dev/i2c-1");
    FAIL() << "Expected NACK to fail the register transfer";
  }
  catch (const std::system_error& error)
  {
    EXPECT_EQ(error.code().value(), EREMOTEIO);
  }

  try
  {
    ValidateI2cTransferResult(-1, EACCES, "/dev/i2c-1");
    FAIL() << "Expected adapter error to fail the register transfer";
  }
  catch (const std::system_error& error)
  {
    EXPECT_EQ(error.code().value(), EACCES);
  }
}

TEST(LinuxI2cDeviceProbe, ClassifiesCommonTransferNacksAsNotPresent)
{
  EXPECT_EQ(ClassifyI2cTransferError(ENXIO), I2cProbeStatus::NotPresent);
  EXPECT_EQ(ClassifyI2cTransferError(EREMOTEIO), I2cProbeStatus::NotPresent);
  EXPECT_EQ(ClassifyI2cTransferError(EIO), I2cProbeStatus::NotPresent);
  EXPECT_EQ(ClassifyI2cTransferError(EACCES), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(EPERM), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(ENOENT), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(EOPNOTSUPP), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(ENOTTY), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(EINVAL), I2cProbeStatus::Error);
  EXPECT_EQ(ClassifyI2cTransferError(EBADF), I2cProbeStatus::Error);
}

TEST(LinuxI2cDeviceProbe, RejectsInvalidAddressWithoutOpeningHardware)
{
  OASIS::PowerMeter::LinuxI2cDeviceProbe probe;
  EXPECT_EQ(probe.Probe("/dev/does-not-exist", 0x02).status,
            OASIS::PowerMeter::I2cProbeStatus::Error);
  EXPECT_EQ(probe.Probe("/dev/does-not-exist", 0x78).status,
            OASIS::PowerMeter::I2cProbeStatus::Error);
}

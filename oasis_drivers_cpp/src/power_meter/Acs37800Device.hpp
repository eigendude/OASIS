/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include "I2cRegisterDevice.hpp"
#include "PowerMeterCore.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace OASIS::PowerMeter
{
// ACS37800-DS Rev. 4, Memory Map and Registers 0x1B, 0x2A, 0x2C, 0x2D
constexpr std::uint8_t ACS37800_SHADOW_CURRENT_CONFIG_REGISTER = 0x1B;
constexpr std::uint8_t ACS37800_INSTANTANEOUS_VOLTAGE_CURRENT_REGISTER = 0x2A;
constexpr std::uint8_t ACS37800_INSTANTANEOUS_POWER_REGISTER = 0x2C;
constexpr std::uint8_t ACS37800_FAULT_STATUS_REGISTER = 0x2D;

/** \brief Physical and expected hardware configuration for one ACS37800 */
struct Acs37800Config
{
  //! Physical current sensing range IPR in amperes: 15, 30, or 90
  double current_sense_range_amps;

  //! Sum of external RISO voltage-divider resistors in ohms
  double voltage_divider_resistance_ohms;

  //! External RSENSE voltage input resistance in ohms
  double voltage_sense_resistance_ohms;

  //! Expected crs_sns shadow-register value in [0, 7]
  //! Used only as a startup consistency check
  unsigned expected_crs_sns;
};

/** \brief Read-only device configuration observed during startup */
struct Acs37800DeviceInfo
{
  //! Raw register 0x1B shadow configuration value
  std::uint32_t current_config_register;

  //! crs_sns value decoded from register 0x1B bits 21:19
  unsigned crs_sns;

  //! Line-to-input voltage multiplier derived from RISO and RSENSE
  double voltage_multiplier;
};

/** \brief Interface used by the node to sample one physical meter */
class IAcs37800Device
{
public:
  virtual ~IAcs37800Device() = default;
  virtual Sample ReadSample() = 0;
  virtual const Acs37800DeviceInfo& GetInfo() const = 0;
};

/** \brief ROS-independent ACS37800 measurement decoder */
class Acs37800Device : public IAcs37800Device
{
public:
  Acs37800Device(std::string device_path, int device_address, const Acs37800Config& config);
  Acs37800Device(std::unique_ptr<II2cRegisterDevice> transport, const Acs37800Config& config);

  Sample ReadSample() override;
  const Acs37800DeviceInfo& GetInfo() const override { return m_info; }

private:
  std::unique_ptr<II2cRegisterDevice> m_transport;
  Acs37800Config m_config;
  Acs37800DeviceInfo m_info{};
};

/** \brief Sign-extend an unsigned field of width 1 through 32 bits */
std::int32_t SignExtend(std::uint32_t value, unsigned width);

/** \brief Validate physical ACS37800 scaling configuration */
void ValidateAcs37800Config(const Acs37800Config& config);

/** \brief Decode signed instantaneous voltage field vcodes to line volts */
double DecodeVoltage(std::uint16_t raw, const Acs37800Config& config);

/** \brief Decode signed instantaneous current field icodes to amperes */
double DecodeCurrent(std::uint16_t raw, const Acs37800Config& config);

/** \brief Decode signed instantaneous power field pinstant to line watts */
double DecodePower(std::uint16_t raw, const Acs37800Config& config);

/** \brief Extract the live overcurrent flag from register 0x2D */
bool DecodeOvercurrent(std::uint32_t raw);
} // namespace OASIS::PowerMeter

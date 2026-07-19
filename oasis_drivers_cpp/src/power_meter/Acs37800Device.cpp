/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "Acs37800Device.hpp"

#include "LinuxI2cRegisterDevice.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <utility>

using OASIS::PowerMeter::Acs37800Config;
using OASIS::PowerMeter::Acs37800Device;

namespace
{
// ACS37800-DS Rev. 4, Electrical Characteristics: instantaneous ADC full-scale
// code at the positive and negative physical input limits
// ACS37800-DS Rev. 4, Electrical Characteristics: differential voltage input
// range is -250 mV through +250 mV
constexpr double VOLTAGE_INPUT_FULL_SCALE_VOLTS = 0.250;

// RMS outputs use normalized fixed-point values. A normalized magnitude of
// 0.84 corresponds to the physical ADC limit, hence the datasheet's 1.19
// conversion factor (1 / 0.84, rounded as documented)
constexpr double RMS_PHYSICAL_SCALE = 1.19;
constexpr double Q16_SCALE = 65536.0;
constexpr double Q15_SCALE = 32768.0;
constexpr double METROLOGY_SAMPLE_RATE_HZ = 32000.0;

// ACS37800-DS Rev. 4, Register 0x1B: crs_sns occupies bits 21:19
constexpr std::uint32_t CRS_SNS_MASK = 0x7U;
constexpr unsigned CRS_SNS_SHIFT = 19;

// Register 0x1F: n is bits 23:14 and bypass_n_en is bit 24
constexpr unsigned RMS_N_SHIFT = 14;
constexpr std::uint32_t RMS_N_MASK = 0x3FFU;
constexpr std::uint32_t BYPASS_N_EN_MASK = 1U << 24U;
constexpr unsigned MAX_COHERENCE_ATTEMPTS = 3;

// ACS37800-DS Rev. 4, Register 0x2D: faultout is bit 1
constexpr std::uint32_t FAULTOUT_MASK = 1U << 1U;

double VoltageMultiplier(const Acs37800Config& config)
{
  // ACS37800-DS Rev. 4, Figures 17 and 18:
  // VLINE / VIN = (sum(RISO) + RSENSE) / RSENSE
  return (config.voltage_divider_resistance_ohms + config.voltage_sense_resistance_ohms) /
         config.voltage_sense_resistance_ohms;
}
} // namespace

std::int32_t OASIS::PowerMeter::SignExtend(std::uint32_t value, unsigned width)
{
  if (width == 0 || width > 32)
    throw std::invalid_argument("signed field width must be in [1, 32]");
  if (width == 32)
    return static_cast<std::int32_t>(value);

  const std::uint32_t field_mask = (1U << width) - 1U;
  const std::uint32_t sign_bit = 1U << (width - 1U);
  value &= field_mask;
  return static_cast<std::int32_t>((value ^ sign_bit) - sign_bit);
}

void OASIS::PowerMeter::ValidateAcs37800Config(const Acs37800Config& config)
{
  if (!std::isfinite(config.current_sense_range_amps) ||
      (config.current_sense_range_amps != 15.0 && config.current_sense_range_amps != 30.0 &&
       config.current_sense_range_amps != 90.0))
  {
    throw std::invalid_argument("current_sense_range_amps must be 15, 30, or 90");
  }
  if (!std::isfinite(config.voltage_divider_resistance_ohms) ||
      config.voltage_divider_resistance_ohms <= 0.0)
  {
    throw std::invalid_argument("voltage_divider_resistance_ohms must be finite and positive");
  }
  if (!std::isfinite(config.voltage_sense_resistance_ohms) ||
      config.voltage_sense_resistance_ohms <= 0.0)
  {
    throw std::invalid_argument("voltage_sense_resistance_ohms must be finite and positive");
  }
  if (config.expected_crs_sns > CRS_SNS_MASK)
    throw std::invalid_argument("expected_crs_sns must be in [0, 7]");
  if (config.rms_sample_count < 4 || config.rms_sample_count > RMS_N_MASK)
    throw std::invalid_argument("rms_sample_count must be in [4, 1023]");
}

double OASIS::PowerMeter::Acs37800VoltageMultiplier(const Acs37800Config& config)
{
  ValidateAcs37800Config(config);
  return VoltageMultiplier(config);
}

double OASIS::PowerMeter::DecodeRmsVoltage(std::uint32_t raw, const Acs37800Config& config)
{
  ValidateAcs37800Config(config);
  const std::uint16_t field = static_cast<std::uint16_t>(raw & 0xFFFFU);
  return static_cast<double>(field) / Q16_SCALE * VOLTAGE_INPUT_FULL_SCALE_VOLTS *
         RMS_PHYSICAL_SCALE * VoltageMultiplier(config);
}

double OASIS::PowerMeter::DecodeRmsCurrent(std::uint32_t raw, const Acs37800Config& config)
{
  ValidateAcs37800Config(config);
  const std::int32_t field = SignExtend((raw >> 16U) & 0xFFFFU, 16);
  return static_cast<double>(field) / Q15_SCALE * config.current_sense_range_amps *
         RMS_PHYSICAL_SCALE;
}

double OASIS::PowerMeter::DecodeActivePower(std::uint32_t raw, const Acs37800Config& config)
{
  ValidateAcs37800Config(config);
  const std::int32_t field = SignExtend(raw & 0xFFFFU, 16);
  return static_cast<double>(field) / Q15_SCALE * VOLTAGE_INPUT_FULL_SCALE_VOLTS *
         config.current_sense_range_amps * VoltageMultiplier(config);
}

bool OASIS::PowerMeter::DecodeOvercurrent(std::uint32_t raw)
{
  return (raw & FAULTOUT_MASK) != 0U;
}

Acs37800Device::Acs37800Device(std::string device_path,
                               int device_address,
                               const Acs37800Config& config)
  : Acs37800Device(std::make_unique<LinuxI2cRegisterDevice>(std::move(device_path), device_address),
                   config)
{
}

Acs37800Device::Acs37800Device(std::unique_ptr<II2cRegisterDevice> transport,
                               const Acs37800Config& config)
  : Acs37800Device(std::move(transport),
                   config,
                   [](double seconds)
                   { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); })
{
}

Acs37800Device::Acs37800Device(std::unique_ptr<II2cRegisterDevice> transport,
                               const Acs37800Config& config,
                               SleepFunction sleep)
  : m_transport(std::move(transport)), m_config(config)
{
  if (!m_transport)
    throw std::invalid_argument("ACS37800 register transport must not be null");
  if (!sleep)
    throw std::invalid_argument("ACS37800 startup sleep function must not be empty");
  ValidateAcs37800Config(m_config);

  m_info.current_config_register =
      m_transport->ReadRegister(ACS37800_SHADOW_CURRENT_CONFIG_REGISTER);
  m_info.crs_sns = (m_info.current_config_register >> CRS_SNS_SHIFT) & CRS_SNS_MASK;
  m_info.voltage_multiplier = VoltageMultiplier(m_config);
  if (m_info.crs_sns != m_config.expected_crs_sns)
  {
    throw std::runtime_error("ACS37800 crs_sns is " + std::to_string(m_info.crs_sns) +
                             ", expected " + std::to_string(m_config.expected_crs_sns));
  }

  std::uint32_t rms_config = m_transport->ReadRegister(ACS37800_SHADOW_RMS_CONFIG_REGISTER);
  rms_config &= ~(RMS_N_MASK << RMS_N_SHIFT);
  rms_config |= (m_config.rms_sample_count << RMS_N_SHIFT) | BYPASS_N_EN_MASK;
  m_transport->WriteRegister(ACS37800_SHADOW_RMS_CONFIG_REGISTER, rms_config);
  const std::uint32_t readback = m_transport->ReadRegister(ACS37800_SHADOW_RMS_CONFIG_REGISTER);
  m_info.bypass_n_en = (readback & BYPASS_N_EN_MASK) != 0U;
  m_info.rms_sample_count = (readback >> RMS_N_SHIFT) & RMS_N_MASK;
  m_info.rms_window_milliseconds =
      static_cast<double>(m_info.rms_sample_count) / METROLOGY_SAMPLE_RATE_HZ * 1000.0;
  if (!m_info.bypass_n_en || m_info.rms_sample_count != m_config.rms_sample_count)
    throw std::runtime_error("ACS37800 fixed RMS configuration readback mismatch");

  // ACS37800-DS Rev. 4 requires 2*n samples before the first fixed-n result is
  // usable. Wait locally so the node cannot publish startup pipeline contents
  sleep(2.0 * static_cast<double>(m_info.rms_sample_count) / METROLOGY_SAMPLE_RATE_HZ);
}

OASIS::PowerMeter::Sample Acs37800Device::ReadSample()
{
  // No result-ready flag, window counter, or multi-register snapshot is
  // documented. Bracketing pactive with RMS reads reduces the chance of
  // combining visibly different windows, but equal quantized RMS values do
  // not prove that all fields came from one metrology window
  std::uint32_t voltage_current = 0;
  std::uint32_t power = 0;
  bool coherent = false;
  for (unsigned attempt = 0; attempt < MAX_COHERENCE_ATTEMPTS; ++attempt)
  {
    ++m_diagnostics.coherent_read_attempts;
    const std::uint32_t before = m_transport->ReadRegister(ACS37800_RMS_VOLTAGE_CURRENT_REGISTER);
    power = m_transport->ReadRegister(ACS37800_ACTIVE_POWER_REGISTER);
    const std::uint32_t after = m_transport->ReadRegister(ACS37800_RMS_VOLTAGE_CURRENT_REGISTER);
    if (before == after)
    {
      voltage_current = after;
      coherent = true;
      if (attempt == 0)
        ++m_diagnostics.successful_first_attempt_reads;
      break;
    }
    ++m_diagnostics.coherence_retries;
  }
  if (!coherent)
  {
    ++m_diagnostics.coherence_retry_exhaustions;
    throw std::runtime_error("ACS37800 coherence retries exhausted");
  }
  const std::uint32_t fault = m_transport->ReadRegister(ACS37800_FAULT_STATUS_REGISTER);

  Sample sample;
  sample.raw_voltage_current_register = voltage_current;
  sample.raw_power_register = power;
  sample.raw_fault_register = fault;
  sample.voltage = DecodeRmsVoltage(voltage_current, m_config);
  sample.current = DecodeRmsCurrent(voltage_current, m_config);
  if (sample.current < 0.0)
    throw std::runtime_error("ACS37800 returned negative irms");
  sample.power = DecodeActivePower(power, m_config);

  // OASIS scalar sensor messages use zero to represent unknown variance. The
  // ACS37800 accuracy limits are not a per-sample statistical covariance
  sample.voltage_variance = 0.0;
  sample.current_variance = 0.0;
  sample.power_variance = 0.0;
  sample.overcurrent = DecodeOvercurrent(fault);
  sample.status = Status::Ok;
  return sample;
}

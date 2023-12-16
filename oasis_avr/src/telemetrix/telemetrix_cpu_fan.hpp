/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include "drivers/cpu_fan_pwm.hpp"
#include "drivers/cpu_fan_tach.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

namespace OASIS
{
/*!
 * \brief Telemetrix subsystem to control a CPU fan
 */
class TelemetrixCPUFan
{
public:
  // Command handlers
  void AttachPWM(uint8_t pwmPin);
  void DetachPWM(uint8_t pwmPin);
  void AttachTachometer(uint8_t tachometerPin);
  void DetachTachometer(uint8_t tachometerPin);
  void SetTachSamplingInterval(uint32_t intervalMs) { m_tachSamplingInterval = intervalMs; }
  void PWMWrite(uint8_t digitalPin, float dutyCycle);

  // Server functions
  void ScanTachometers();
  void ResetData();

private:
  // CPU fan drivers
  CPUFanPWM m_cpuFanPWM;
  CPUFanTach m_cpuFanTach;

  // Timing parameters
  Timer m_sampleTimer;
  uint32_t m_tachSamplingInterval{0};
};
} // namespace OASIS

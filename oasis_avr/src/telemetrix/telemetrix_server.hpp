/*
 *  Copyright (C) 2022 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS
{
class TelemetrixCPUFan;
class TelemetrixDHT;
class TelemetrixI2C;
class TelemetrixMemory;
class TelemetrixOneWire;
class TelemetrixPins;
class TelemetrixServo;
class TelemetrixSonar;
class TelemetrixSPI;
class TelemetrixStepper;

class TelemetrixServer
{
public:
  void Setup();
  void Loop();

  uint8_t Features() const { return m_features; }

  void SetReporting(bool reporting) { m_stopReports = !reporting; }

  // Subsystems
  TelemetrixCPUFan* GetCPUFan() const { return m_cpuFan; }
  TelemetrixDHT* GetDHT() const { return m_dht; }
  TelemetrixI2C* GetI2C() const { return m_i2c; }
  TelemetrixMemory* GetMemory() const { return m_memory; }
  TelemetrixOneWire* GetOneWire() const { return m_oneWire; }
  TelemetrixPins* GetPins() const { return m_pins; }
  TelemetrixServo* GetServo() const { return m_servo; }
  TelemetrixSonar* GetSonar() const { return m_sonar; }
  TelemetrixSPI* GetSPI() const { return m_spi; }
  TelemetrixStepper* GetStepper() const { return m_stepper; }

private:
  void ScanDigitalInputs();
  void ScanAnalogInputs();
  void ScanMemory();
  void ScanCPUFans();
  void ScanSonars();
  void ScanDHTs();
  void RunSteppers();

  // Subsystems
  TelemetrixCPUFan* m_cpuFan{nullptr};
  TelemetrixDHT* m_dht{nullptr};
  TelemetrixI2C* m_i2c{nullptr};
  TelemetrixMemory* m_memory{nullptr};
  TelemetrixOneWire* m_oneWire{nullptr};
  TelemetrixPins* m_pins{nullptr};
  TelemetrixServo* m_servo{nullptr};
  TelemetrixSonar* m_sonar{nullptr};
  TelemetrixSPI* m_spi{nullptr};
  TelemetrixStepper* m_stepper{nullptr};

  // A byte to hold the enabled features. The masks are OR'ed into the features
  // byte. Set during call to Setup().
  uint8_t m_features{0};

  // A flag to stop sending all report messages
  bool m_stopReports{false};
};
} // namespace OASIS

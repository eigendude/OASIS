/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from FirmataExpress under the AGPL 3.0 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "utils/timer.hpp"

#include <stdint.h>

#include <Boards.h>

namespace OASIS
{

// Forward-declare subsystems
class FirmataAnalog;
class FirmataBluefruit;
class FirmataCPUFan;
class FirmataDHT;
class FirmataDiagnostics;
class FirmataDigital;
class FirmataI2C;
class FirmataServo;
class FirmataSonar;
class FirmataSPI;
class FirmataStepper;
class FirmataSubsystem;

class FirmataThread
{
private:
  /*!
   * \brief Create the thread
   */
  FirmataThread();

public:
  static FirmataThread& GetInstance();

  // Lifecycle functions
  void Setup();
  void Reset() {} // TODO

  // Subsystems (const)
  const FirmataAnalog* GetAnalog() const { return m_analog; }
  const FirmataBluefruit* GetBluefruit() const { return m_bluefruit; }
  const FirmataCPUFan* GetCPUFan() const { return m_cpuFan; }
  const FirmataDHT* GetDHT() const { return m_dht; }
  const FirmataDiagnostics* GetDiagnostics() const { return m_diagnostics; }
  const FirmataDigital* GetDigital() const { return m_digital; }
  const FirmataI2C* GetI2C() const { return m_i2c; }
  const FirmataServo* GetServo() const { return m_servo; }
  const FirmataSonar* GetSonar() const { return m_sonar; }
  const FirmataSPI* GetSPI() const { return m_spi; }
  const FirmataStepper* GetStepper() const { return m_stepper; }

  // Subsystems (mutable)
  FirmataAnalog* GetAnalog() { return m_analog; }
  FirmataBluefruit* GetBluefruit() { return m_bluefruit; }
  FirmataCPUFan* GetCPUFan() { return m_cpuFan; }
  FirmataDHT* GetDHT() { return m_dht; }
  FirmataDiagnostics* GetDiagnostics() { return m_diagnostics; }
  FirmataDigital* GetDigital() { return m_digital; }
  FirmataI2C* GetI2C() { return m_i2c; }
  FirmataServo* GetServo() { return m_servo; }
  FirmataSonar* GetSonar() { return m_sonar; }
  FirmataSPI* GetSPI() { return m_spi; }
  FirmataStepper* GetStepper() { return m_stepper; }

  // Timer functions
  void SetSamplingInterval(uint32_t samplingIntervalMs)
  {
    m_samplingIntervalMs = samplingIntervalMs;
  }

private:
  enum SubsystemID
  {
    ANALOG,
    BLUEFRUIT,
    CPU_FAN,
    DHT,
    DIAGNOSTICS,
    DIGITAL,
    I2C,
    SERVO,
    SONAR,
    SPI,
    STEPPER,

    // Leave this last to get a count of subsystems
    SUBSYSTEM_COUNT
  };

  // Threading functions
  void MessageLoop();
  void SamplingLoop();

  // Static threading functions
  static void FirmataMessageLoop();
  static void FirmataSamplingLoop();

  // Subsystems
  FirmataAnalog* m_analog{nullptr};
  FirmataBluefruit* m_bluefruit{nullptr};
  FirmataCPUFan* m_cpuFan{nullptr};
  FirmataDHT* m_dht{nullptr};
  FirmataDiagnostics* m_diagnostics{nullptr};
  FirmataDigital* m_digital{nullptr};
  FirmataI2C* m_i2c{nullptr};
  FirmataServo* m_servo{nullptr};
  FirmataSonar* m_sonar{nullptr};
  FirmataSPI* m_spi{nullptr};
  FirmataStepper* m_stepper{nullptr};

  FirmataSubsystem* m_subsystems[SubsystemID::SUBSYSTEM_COUNT]{};

  // Timing parameters
  Timer m_samplingTimer;
  uint32_t m_samplingIntervalMs{0};
};

} // namespace OASIS

/*
 *  Copyright (C) 2021-2025 Garrett Brown
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

#include "firmata_thread.hpp"

#include "firmata_callbacks.hpp"
#include "firmata_subsystem.hpp"

#if defined(ENABLE_ANALOG)
#include "firmata_analog.hpp"
#endif

#if defined(ENABLE_BLUEFRUIT)
#include "firmata_bluefruit.hpp"
#endif

#if defined(ENABLE_CPU_FAN)
#include "firmata_cpu_fan.hpp"
#endif

#if defined(ENABLE_DHT)
#include "firmata_dht.hpp"
#endif

#if defined(ENABLE_DIAGNOSTICS)
#include "firmata_diagnostics.hpp"
#endif

#if defined(ENABLE_DIGITAL)
#include "firmata_digital.hpp"
#endif

#if defined(ENABLE_I2C)
#include "firmata_i2c.hpp"
#endif

#if defined(ENABLE_SERVO)
#include "firmata_servo.hpp"
#endif

#if defined(ENABLE_SONAR)
#include "firmata_sonar.hpp"
#endif

#if defined(ENABLE_SPI)
#include "firmata_spi.hpp"
#endif

#if defined(ENABLE_STEPPER)
#include "firmata_stepper.hpp"
#endif

#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Serial constants
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Threading constants
constexpr size_t FIRMATA_MESSAGING_STACK_SIZE = 256; // Default is 128
constexpr size_t FIRMATA_SAMPLING_STACK_SIZE = 96; // Default is 128

} // namespace OASIS

FirmataThread::FirmataThread()
{
#if defined(ENABLE_ANALOG)
  static FirmataAnalog analog;
  m_subsystems[SubsystemID::ANALOG] = m_analog = &analog;
#endif

#if defined(ENABLE_BLUEFRUIT)
  static FirmataBluefruit bluefruit;
  m_subsystems[SubsystemID::BLUEFRUIT] = m_bluefruit = &bluefruit;
#endif

#if defined(ENABLE_DHT)
  static FirmataDHT dht;
  m_subsystems[SubsystemID::DHT] = m_dht = &dht;
#endif

#if defined(ENABLE_DIAGNOSTICS)
  static FirmataDiagnostics diagnostics;
  m_subsystems[SubsystemID::DIAGNOSTICS] = m_diagnostics = &diagnostics;
#endif

#if defined(ENABLE_DIGITAL)
  static FirmataDigital digital;
  m_subsystems[SubsystemID::DIGITAL] = m_digital = &digital;
#endif

#if defined(ENABLE_I2C)
  static FirmataI2C i2c;
  m_subsystems[SubsystemID::I2C] = m_i2c = &i2c;
#endif

#if defined(ENABLE_SERVO)
  static FirmataServo servo;
  m_subsystems[SubsystemID::SERVO] = m_servo = &servo;
#endif

#if defined(ENABLE_SONAR)
  static FirmataSonar sonar;
  m_subsystems[SubsystemID::SONAR] = m_sonar = &sonar;
#endif

#if defined(ENABLE_SPI)
  static FirmataSPI spi;
  m_subsystems[SubsystemID::SPI] = m_spi = &spi;
#endif

#if defined(ENABLE_STEPPER)
  static FirmataStepper stepper;
  m_subsystems[SubsystemID::STEPPER] = m_stepper = &stepper;
#endif
}

FirmataThread& FirmataThread::GetInstance()
{
  static FirmataThread instance;
  return instance;
}

void FirmataThread::Setup()
{
  // Set Firmata version
  Firmata.setFirmwareNameAndVersion("OASIS", FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  // Initialize callbacks
  FirmataCallbacks::InitializeCallbacks(*this);

  // Skip the call to blinkVersion, normally called inside Firmata.begin(baud)
  Firmata.disableBlinkVersion();

  // Initialize the default Serial transport and override the default baud
  Firmata.begin(SERIAL_BAUD_RATE);

  // Start Firmata messaging thread
  Scheduler.startLoop(FirmataMessageLoop, FIRMATA_MESSAGING_STACK_SIZE);

  // Configure subsystems
  for (unsigned int i = 0; i < SubsystemID::SUBSYSTEM_COUNT; ++i)
  {
    if (m_subsystems[i] != nullptr)
      m_subsystems[i]->Setup();
  }

  // Start Firmata sampling thread
  Scheduler.startLoop(FirmataSamplingLoop, FIRMATA_SAMPLING_STACK_SIZE);
}

void FirmataThread::MessageLoop()
{
  // Processing incoming message as soon as possible
  while (Firmata.available())
    Firmata.processInput();

  // Loop subsystems
  for (unsigned int i = 0; i < SubsystemID::SUBSYSTEM_COUNT; ++i)
  {
    if (m_subsystems[i] != nullptr)
      m_subsystems[i]->Loop();
  }

  // TODO: Ensure that Stream buffer doesn't go over 60 bytes
  yield();
}

void FirmataThread::SamplingLoop()
{
  if (m_samplingIntervalMs > 0)
  {
    m_samplingTimer.SetTimeout(m_samplingIntervalMs);

    // Sample subsystems
    for (unsigned int i = 0; i < SubsystemID::SUBSYSTEM_COUNT; ++i)
    {
      if (m_subsystems[i] != nullptr)
      {
        m_subsystems[i]->Sample();
        yield();
      }
    }

    delay(m_samplingTimer.TimeLeft());
  }
  else
  {
    yield();
  }
}

void FirmataThread::FirmataMessageLoop()
{
  GetInstance().MessageLoop();
}

void FirmataThread::FirmataSamplingLoop()
{
  GetInstance().SamplingLoop();
}

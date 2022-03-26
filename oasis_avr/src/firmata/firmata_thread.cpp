/*
 *  Copyright (C) 2021 Garrett Brown
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

#if defined(ENABLE_ANALOG)
#include "firmata_analog.hpp"
#endif

#if defined(ENABLE_DIAGNOSTICS)
#include "firmata_diagnostics.hpp"
#endif

#if defined(ENABLE_DIGITAL)
#include "firmata_digital.hpp"
#endif

#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Serial constants
static constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Timing constants
// The minimum interval for sampling analog input
static constexpr uint8_t MINIMUM_SAMPLING_INTERVAL = 1;

// Threading constants
constexpr size_t FIRMATA_STACK_SIZE = 96; // Default is 128

} // namespace OASIS

FirmataThread::FirmataThread()
{
#if defined(ENABLE_ANALOG)
  static FirmataAnalog analog;
  m_analog = &analog;
#endif

#if defined(ENABLE_DIAGNOSTICS)
  static FirmataDiagnostics diagnostics;
  m_diagnostics = &diagnostics;
#endif

#if defined(ENABLE_DIGITAL)
  static FirmataDigital digital;
  m_digital = &digital;
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
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  // Initialize callbacks
  FirmataCallbacks::InitializeCallbacks(*this);

  // Skip the call to blinkVersion, normally called inside Firmata.begin(baud)
  Firmata.disableBlinkVersion();

  // Initialize the default Serial transport and override the default baud
  Firmata.begin(SERIAL_BAUD_RATE);

  // Start Firmata thread
  Scheduler.startLoop(FirmataLoop, FIRMATA_STACK_SIZE);

  // Configure subsystems
#if defined(ENABLE_ANALOG)
  m_analog->Setup(AnalogLoop);
#endif

#if defined(ENABLE_DIAGNOSTICS)
  m_diagnostics->Setup(DiagnosticsLoop);
#endif

#if defined(ENABLE_DIGITAL)
  m_digital->Setup(DigitalLoop);
#endif
}

void FirmataThread::Reset()
{
  m_isResetting = true;

  // Initialize a default state
  // TODO: Option to load config from EEPROM instead of default

  // Reset subsystems
#if defined(ENABLE_ANALOG)
  m_analog->Reset();
#endif

#if defined(ENABLE_DIAGNOSTICS)
  m_diagnostics->Reset();
#endif

#if defined(ENABLE_DIGITAL)
  m_digital->Reset();
#endif

  /* TODO
  for (uint8_t i = 0; i < TOTAL_PINS; + i)
  {
    // Pins with analog capability default to analog input. Otherwise, pins
    // default to digital output
    if (IS_PIN_ANALOG(i))
    {
      // Turns off pullup, configures everything
      FirmataCallbacks::SetPinModeCallback(i, PIN_MODE_ANALOG); // TODO
    }
    else
    {
      // Sets the output to 0, configures portConfigInputs
      FirmataCallbacks::SetPinModeCallback(i, OUTPUT); // TODO
    }

    m_servoPinMap[i] = 255;
  }
  */

  // Done resetting
  m_isResetting = false;
}

void FirmataThread::Loop()
{
  // Processing incoming message as soon as possible
  while (Firmata.available())
    Firmata.processInput();

  // TODO: Ensure that Stream buffer doesn't go over 60 bytes
  yield();
}

void FirmataThread::SetSamplingInterval(uint8_t samplingIntervalMs)
{
  // Validate parameters
  if (samplingIntervalMs < MINIMUM_SAMPLING_INTERVAL)
    samplingIntervalMs = MINIMUM_SAMPLING_INTERVAL;

  // Update state
  m_samplingIntervalMs = samplingIntervalMs;
}

void FirmataThread::FirmataLoop()
{
  GetInstance().Loop();
}

void FirmataThread::AnalogLoop()
{
#if defined(ENABLE_ANALOG)
  GetInstance().GetAnalog()->Loop();
#endif
}

void FirmataThread::DiagnosticsLoop()
{
#if defined(ENABLE_DIAGNOSTICS)
  GetInstance().GetDiagnostics()->Loop();
#endif
}

void FirmataThread::DigitalLoop()
{
#if defined(ENABLE_DIGITAL)
  GetInstance().GetDigital()->Loop();
#endif
}

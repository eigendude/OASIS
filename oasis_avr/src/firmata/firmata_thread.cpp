/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the FirmataExpress project and the AGPL-3. License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_thread.hpp"

#include "firmata_analog.hpp"
#include "firmata_callbacks.hpp"
#include "firmata_digital.hpp"

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
constexpr size_t FIRMATA_STACK_SIZE = 64; // Default is 128

} // namespace OASIS

FirmataThread::FirmataThread(FirmataAnalog& analog, FirmataDigital& digital)
  : m_analog(analog), m_digital(digital)
{
}

FirmataThread& FirmataThread::GetInstance()
{
  // Subsystem storage
  static FirmataAnalog analog;
  static FirmataDigital digital;

  // Instance storage
  static FirmataThread instance(analog, digital);

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
  m_analog.Setup(AnalogLoop);
  m_digital.Setup(DigitalLoop);
}

void FirmataThread::Reset()
{
  m_isResetting = true;

  // Initialize a default state
  // TODO: Option to load config from EEPROM instead of default

  // Reset subsystems
  m_analog.Reset();
  m_digital.Reset();

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

void FirmataThread::KeepAlive(unsigned int newKeepAliveIntervalSecs)
{
  // Update state
  m_keepAliveIntervalSecs = newKeepAliveIntervalSecs;
  m_previousKeepAliveMs = millis();
}

void FirmataThread::FirmataLoop()
{
  GetInstance().Loop();
}

void FirmataThread::AnalogLoop()
{
  GetInstance().GetAnalog().Loop();
}

void FirmataThread::DigitalLoop()
{
  GetInstance().GetDigital().Loop();
}

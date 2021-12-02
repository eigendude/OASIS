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

#include "firmata_callbacks.hpp"

#include "firmata_analog.hpp"
#include "firmata_digital.hpp"
#include "firmata_thread.hpp"
#include "firmata_utils.hpp"

#include <Arduino.h>
#include <Boards.h>
#include <FirmataExpress.h>

using namespace OASIS;

namespace OASIS
{

// Firmata constants
constexpr uint8_t ARDUINO_INSTANCE_ID = 1;

} // namespace OASIS

FirmataThread* FirmataCallbacks::m_thread = nullptr;

void FirmataCallbacks::InitializeCallbacks(FirmataThread& thread)
{
  // Initialize state
  m_thread = &thread;

  // Initialize Firmata
  Firmata.attach(ANALOG_MESSAGE, AnalogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, DigitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, ReportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, ReportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, SetPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, SetPinValueCallback);
  Firmata.attach(START_SYSEX, SysexCallback);
  Firmata.attach(SYSTEM_RESET, SystemResetCallback);
}

void FirmataCallbacks::AnalogWriteCallback(uint8_t pin, int analogValue)
{
  if (pin < TOTAL_PINS)
  {
    switch (Firmata.getPinMode(pin))
    {
      case PIN_MODE_PWM:
      {
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), analogValue);

        Firmata.setPinState(pin, analogValue);

        break;
      }

      default:
        break;
    }
  }
}

void FirmataCallbacks::DigitalWriteCallback(uint8_t digitalPort, int portValue)
{
  uint8_t mask = 1;
  uint8_t pinWriteMask = 0;

  if (digitalPort < TOTAL_PORTS)
  {
    // Create a mask of the pins on this port that are writable.
    uint8_t lastPin = digitalPort * 8 + 8;

    if (lastPin > TOTAL_PINS)
      lastPin = TOTAL_PINS;

    for (uint8_t pin = digitalPort * 8; pin < lastPin; ++pin)
    {
      // Do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin))
      {
        // Do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT || Firmata.getPinMode(pin) == PIN_MODE_INPUT)
        {
          uint8_t pinValue = (static_cast<uint8_t>(portValue) & mask) ? 1 : 0;

          if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT)
          {
            pinWriteMask |= mask;
          }
          else if (Firmata.getPinMode(pin) == PIN_MODE_INPUT && pinValue == 1 &&
                   Firmata.getPinState(pin) != 1)
          {
            // Only handle INPUT here for backwards compatibility
            pinMode(pin, INPUT_PULLUP);
          }

          Firmata.setPinState(pin, pinValue);
        }
      }

      mask = mask << 1;
    }

    writePort(digitalPort, static_cast<uint8_t>(portValue), pinWriteMask);
  }
}

void FirmataCallbacks::ReportAnalogCallback(uint8_t analogPin, int enableReporting)
{
  if (analogPin < TOTAL_ANALOG_PINS)
  {
    m_thread->GetAnalog().EnableAnalogInput(analogPin, enableReporting != 0);

    if (enableReporting != 0)
    {
      // Prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!m_thread->IsResetting())
      {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, WiFi or Bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }

  // TODO: Save status to EEPROM here, if changed
}

void FirmataCallbacks::ReportDigitalCallback(uint8_t digitalPort, int enableReporting)
{
  if (digitalPort < TOTAL_PORTS)
  {
    m_thread->GetDigital().EnableDigitalInput(digitalPort, enableReporting != 0);

    // Send port value immediately. This is helpful when connected via ethernet,
    // WiFi or Bluetooth so pin states can be known upon reconnecting.
    if (enableReporting != 0)
      m_thread->GetDigital().SendPort(digitalPort);
  }

  // Do not disable analog reporting on these 8 pins, to allow some pins to be
  // used for digital, others analog.
  //
  // Instead, allow both types of reporting to be enabled, but check if the
  // pin is configured as analog when sampling the analog inputs.
  //
  // Likewise, while scanning digital pins, portConfigInputs will mask off
  // values from any pins configured as analog.
}

void FirmataCallbacks::SetPinModeCallback(uint8_t pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (IS_PIN_ANALOG(pin))
  {
    // Turn on/off reporting
    ReportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0);
  }

  if (IS_PIN_DIGITAL(pin))
  {
    m_thread->GetDigital().SetDigitalPinMode(pin, mode);
  }

  // Update Firmata state
  Firmata.setPinState(pin, 0);

  // Handle pin mode change
  switch (mode)
  {
    case PIN_MODE_ANALOG:
    {
      if (IS_PIN_ANALOG(pin))
      {
        if (IS_PIN_DIGITAL(pin))
        {
          // Disable output driver
          pinMode(PIN_TO_DIGITAL(pin), INPUT);
        }

        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    }

    case PIN_MODE_INPUT:
    {
      if (IS_PIN_DIGITAL(pin))
      {
        // Disable output driver
        pinMode(PIN_TO_DIGITAL(pin), INPUT);

        Firmata.setPinMode(pin, PIN_MODE_INPUT);
      }
      break;
    }

    case PIN_MODE_PULLUP:
    {
      if (IS_PIN_DIGITAL(pin))
      {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);

        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    }

    case PIN_MODE_OUTPUT:
    {
      if (IS_PIN_DIGITAL(pin))
      {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM)
        {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }

        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);

        Firmata.setPinMode(pin, PIN_MODE_OUTPUT);
      }
      break;
    }

    case PIN_MODE_PWM:
    {
      if (IS_PIN_PWM(pin))
      {
        pinMode(PIN_TO_PWM(pin), OUTPUT);

        analogWrite(PIN_TO_PWM(pin), 0);

        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    }

    default:
    {
      // TODO: Put error msgs in EEPROM
      Firmata.sendString("Unknown pin mode");
      break;
    }
  }

  // TODO: Save status to EEPROM here, if changed
}

void FirmataCallbacks::SetPinValueCallback(uint8_t pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin))
  {
    if (Firmata.getPinMode(pin) == PIN_MODE_OUTPUT)
    {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void FirmataCallbacks::SysexCallback(uint8_t command, uint8_t argc, uint8_t* argv)
{
  switch (command)
  {
    case RU_THERE:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(static_cast<uint8_t>(I_AM_HERE));
      Firmata.write(static_cast<uint8_t>(ARDUINO_INSTANCE_ID));
      Firmata.write(END_SYSEX);

      break;
    }

    case KEEP_ALIVE:
    {
      const unsigned int newKeepAliveIntervalSecs = argv[0] + (argv[1] << 7);

      m_thread->KeepAlive(newKeepAliveIntervalSecs);

      break;
    }

    case SAMPLING_INTERVAL:
    {
      if (argc > 1)
      {
        const uint8_t samplingIntervalMs = argv[0] + (argv[1] << 7);
        m_thread->SetSamplingInterval(samplingIntervalMs);
      }
      else
      {
        // This was commented in FirmataExpress.ino
        //Firmata.sendString("Not enough data");
      }

      break;
    }

    case EXTENDED_ANALOG:
    {
      if (argc > 1)
      {
        int val = argv[1];

        if (argc > 2)
          val |= (argv[2] << 7);

        if (argc > 3)
          val |= (argv[3] << 14);

        AnalogWriteCallback(argv[0], val);
      }

      break;
    }

    case CAPABILITY_QUERY:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (uint8_t pin = 0; pin < TOTAL_PINS; ++pin)
      {
        if (IS_PIN_DIGITAL(pin))
        {
          Firmata.write(static_cast<uint8_t>(PIN_MODE_INPUT));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_PULLUP));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_OUTPUT));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_STEPPER));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_SONAR));
          Firmata.write(1);
          Firmata.write(static_cast<uint8_t>(PIN_MODE_DHT));
          Firmata.write(1);
        }

        if (IS_PIN_ANALOG(pin))
        {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }

        if (IS_PIN_PWM(pin))
        {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }

        if (IS_PIN_DIGITAL(pin))
        {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }

        if (IS_PIN_I2C(pin))
        {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1); // TODO: Could assign a number to map to SCL or SDA
        }

        Firmata.write(127);
      }

      Firmata.write(END_SYSEX);

      break;
    }

    case PIN_STATE_QUERY:
    {
      if (argc > 0)
      {
        uint8_t pin = argv[0];

        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);

        if (pin < TOTAL_PINS)
        {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin)) & 0x7F);

          if (Firmata.getPinState(pin) & 0xFF80)
            Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin) >> 7) & 0x7F);

          if (Firmata.getPinState(pin) & 0xC000)
            Firmata.write(static_cast<uint8_t>(Firmata.getPinState(pin) >> 14) & 0x7F);
        }

        Firmata.write(END_SYSEX);
      }

      break;
    }

    case ANALOG_MAPPING_QUERY:
    {
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);

      for (uint8_t pin = 0; pin < TOTAL_PINS; ++pin)
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);

      Firmata.write(END_SYSEX);

      break;
    }

    default:
      break;
  }
}

void FirmataCallbacks::SystemResetCallback()
{
  m_thread->Reset();
}

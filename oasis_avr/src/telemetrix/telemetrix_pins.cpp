/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_pins.hpp"

#include "telemetrix_reports.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>

using namespace OASIS;

namespace
{
// To translate a pin number from an integer value to its analog pin number
// equivalent, this array is used to look up the value to use for the pin.
#ifdef ARDUINO_SAMD_MKRWIFI1010
static const int analogReadPins[20] = {A0, A1, A2, A3, A4, A5, A6};
#else
static const int analogReadPins[20] = {A0, A1, A2,  A3,  A4,  A5,  A6,  A7,
                                       A8, A9, A10, A11, A12, A13, A14, A15};
#endif
} // namespace

TelemetrixPins::TelemetrixPins()
{
  InitPinStructures();
}

void TelemetrixPins::InitPinStructures()
{
  // Initialize the digital pin array
  for (unsigned int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
  {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = 0;
  }

  // Initialize the analog pin array
  for (unsigned int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
  {
    the_analog_pins[i].pin_number = i;
    the_analog_pins[i].pin_mode = AT_MODE_NOT_SET;
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = 0;
    the_analog_pins[i].differential = 0;
  }
}

void TelemetrixPins::scan_digital_inputs()
{
  //
  // Report message
  //
  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = value
  //
  uint8_t report_message[4] = {3, DIGITAL_REPORT, 0, 0};

  for (unsigned int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; ++i)
  {
    if (the_digital_pins[i].pin_mode == INPUT || the_digital_pins[i].pin_mode == INPUT_PULLUP)
    {
      if (the_digital_pins[i].reporting_enabled)
      {
        // If the value changed since last read
        const uint8_t value = static_cast<uint8_t>(digitalRead(the_digital_pins[i].pin_number));
        if (value != the_digital_pins[i].last_value)
        {
          the_digital_pins[i].last_value = value;
          report_message[2] = static_cast<uint8_t>(i);
          report_message[3] = value;
          Serial.write(report_message, 4);
        }
      }
    }
  }
}

void TelemetrixPins::scan_analog_inputs()
{
  //
  // Report message
  //
  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value
  //
  uint8_t report_message[5] = {4, ANALOG_REPORT, 0, 0, 0};

  const unsigned long current_millis = millis();
  if (current_millis - previous_millis > analog_sampling_interval)
  {
    previous_millis += analog_sampling_interval;

    for (unsigned int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; ++i)
    {
      if (the_analog_pins[i].pin_mode == AT_ANALOG)
      {
        if (the_analog_pins[i].reporting_enabled)
        {
          // If the value changed since last read adjust pin number for the
          // actual read
          const uint8_t adjusted_pin_number = static_cast<uint8_t>(analogReadPins[i]);
          const int value = analogRead(adjusted_pin_number);
          const int differential = abs(value - the_analog_pins[i].last_value);

          if (differential >= the_analog_pins[i].differential)
          {
            // Trigger value achieved, send out the report
            the_analog_pins[i].last_value = value;

            report_message[2] = static_cast<uint8_t>(i);
            report_message[3] = highByte(value); // Get high order byte
            report_message[4] = lowByte(value);

            Serial.write(report_message, 5);
            delay(1);
          }
        }
      }
    }
  }
}

void TelemetrixPins::DigitalWrite(uint8_t pin, uint8_t value)
{
  digitalWrite(pin, value);
}

void TelemetrixPins::AnalogWrite(uint8_t pin, unsigned int value)
{
  analogWrite(pin, static_cast<int>(value));
}

void TelemetrixPins::set_pin_mode_input(uint8_t pin, bool reportingEnabled)
{
  the_digital_pins[pin].pin_mode = INPUT;
  the_digital_pins[pin].reporting_enabled = reportingEnabled;
  pinMode(pin, INPUT);
}

void TelemetrixPins::set_pin_mode_input_pullup(uint8_t pin, bool reportingEnabled)
{
  the_digital_pins[pin].pin_mode = INPUT_PULLUP;
  the_digital_pins[pin].reporting_enabled = reportingEnabled;
  pinMode(pin, INPUT_PULLUP);
}

void TelemetrixPins::set_pin_mode_output(uint8_t pin)
{
  the_digital_pins[pin].pin_mode = OUTPUT;
  pinMode(pin, OUTPUT);
}

void TelemetrixPins::set_pin_mode_analog(uint8_t pin, int differential, bool reportingEnabled)
{
  the_analog_pins[pin].pin_mode = AT_ANALOG;
  the_analog_pins[pin].differential = differential;
  the_analog_pins[pin].reporting_enabled = reportingEnabled;
}

void TelemetrixPins::set_analog_sampling_interval(uint8_t analogSamplingInterval)
{
  analog_sampling_interval = analogSamplingInterval;
}

void TelemetrixPins::modify_reporting(uint8_t pin, uint8_t reporting)
{
  switch (reporting)
  {
    case REPORTING_DISABLE_ALL:
    {
      for (unsigned int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++)
        the_digital_pins[i].reporting_enabled = false;
      for (unsigned int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++)
        the_analog_pins[i].reporting_enabled = false;
      break;
    }
    case REPORTING_ANALOG_ENABLE:
    {
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
        the_analog_pins[pin].reporting_enabled = true;
      break;
    }
    case REPORTING_ANALOG_DISABLE:
    {
      if (the_analog_pins[pin].pin_mode != AT_MODE_NOT_SET)
        the_analog_pins[pin].reporting_enabled = false;
      break;
    }
    case REPORTING_DIGITAL_ENABLE:
    {
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
        the_digital_pins[pin].reporting_enabled = true;
      break;
    }
    case REPORTING_DIGITAL_DISABLE:
    {
      if (the_digital_pins[pin].pin_mode != AT_MODE_NOT_SET)
        the_digital_pins[pin].reporting_enabled = false;
      break;
    }
    default:
      break;
  }
}

void TelemetrixPins::reset_data()
{
  previous_millis = 0;
  analog_sampling_interval = 19;
}

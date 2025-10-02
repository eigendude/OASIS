/*
 *  Copyright (C) 2022-2025 Garrett Brown
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

#include <Arduino.h>

// Pin mode definitions

// INPUT defined in Arduino.h = 0
// OUTPUT defined in Arduino.h = 1
// INPUT_PULLUP defined in Arduino.h = 2
// The following are defined for arduino_telemetrix (AT)
#define AT_ANALOG 3
#define AT_MODE_NOT_SET 255

// Maximum number of pins supported
#define MAX_DIGITAL_PINS_SUPPORTED 100
#define MAX_ANALOG_PINS_SUPPORTED 15

// Analog input pin numbers are defined from
// A0 - A7. Since we do not know if the board
// in use also supports higher analog pin numbers
// we need to define those pin numbers to allow
// the program to compile, even though the
// pins may not exist for the board in use.

#ifndef A6
#define A6 2047
#endif

#ifndef A7
#define A7 2047
#endif

#ifndef A8
#define A8 2047
#endif

#ifndef A9
#define A9 2047
#endif

#ifndef A10
#define A10 2047
#endif

#ifndef PIN_A11
#define A11 2047
#endif

#ifndef PIN_A12
#define A12 2047
#endif

#ifndef PIN_A13
#define A13 2047
#endif

#ifndef PIN_A14
#define A14 2047
#endif

#ifndef PIN_A15
#define A15 2047
#endif

namespace OASIS
{
class TelemetrixPins
{
public:
  TelemetrixPins();

  void InitPinStructures();

  void scan_analog_inputs();
  void scan_digital_inputs();

  void DigitalWrite(uint8_t pin, uint8_t value);
  void AnalogWrite(uint8_t pin, unsigned int value);

  void set_pin_mode_input(uint8_t pin, bool reportingEnabled);
  void set_pin_mode_input_pullup(uint8_t pin, bool reportingEnabled);
  void set_pin_mode_output(uint8_t pin);
  void set_pin_mode_analog(uint8_t pin, int differential, bool reportingEnabled);
  void set_analog_sampling_interval(uint8_t analogSamplingInterval);

  void modify_reporting(uint8_t pin, uint8_t reporting);

  void reset_data();

private:
  //////////////////////////////////////////////////////////////////////////////
  // Digital pins
  //////////////////////////////////////////////////////////////////////////////

  // A descriptor for digital pins
  struct pin_descriptor
  {
    uint8_t pin_number;
    uint8_t pin_mode;

    // If true, then send reports if an input pin
    bool reporting_enabled;

    // Last value read for input mode
    int last_value;
  };

  // An array of digital_pin_descriptors
  pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

  //////////////////////////////////////////////////////////////////////////////
  // Analog pint
  //////////////////////////////////////////////////////////////////////////////

  // A descriptor for analog pins
  struct analog_pin_descriptor
  {
    uint8_t pin_number;
    uint8_t pin_mode;

    // If true, then send reports if an input pin
    bool reporting_enabled;

    // Last value read for input mode
    int last_value;

    // Difference between current and last value needed to generate a report
    int differential;
  };

  // An array of analog_pin_descriptors
  analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

  // Analog sampling interval
  uint8_t analog_sampling_interval{19};

  // For analog input loop
  unsigned long previous_millis{0};
};
} // namespace OASIS

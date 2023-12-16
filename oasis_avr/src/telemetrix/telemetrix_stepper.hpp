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

#include <stdint.h>

#define MAX_NUMBER_OF_STEPPERS 4

class AccelStepper;

namespace OASIS
{
class TelemetrixStepper
{
public:
  TelemetrixStepper();

  void RunSteppers();

  void set_pin_mode_stepper(uint8_t motorId,
                            uint8_t interface,
                            uint8_t pin1,
                            uint8_t pin2,
                            uint8_t pin3,
                            uint8_t pin4,
                            bool enable);
  void stepper_move_to(uint8_t motorId,
                       uint8_t positionMSB,
                       uint8_t positionMSB1,
                       uint8_t positionMSB2,
                       uint8_t positionLSB,
                       bool polarity);
  void stepper_move(uint8_t motorId,
                    uint8_t positionMSB,
                    uint8_t positionMSB1,
                    uint8_t positionMSB2,
                    uint8_t positionLSB,
                    bool polarity);
  void stepper_run(uint8_t motorId);
  void stepper_run_speed(uint8_t motorId);
  void stepper_set_max_speed(uint8_t motorId, uint8_t speedMSB, uint8_t speedLSB);
  void stepper_set_acceleration(uint8_t motorId, uint8_t accelMSB, uint8_t accelLSB);
  void stepper_set_speed(uint8_t motorId, uint8_t speedMSB, uint8_t speedLSB);
  void stepper_get_distance_to_go(uint8_t motorId);
  void stepper_get_target_position(uint8_t motorId);
  void stepper_get_current_position(uint8_t motorId);
  void stepper_set_current_position(uint8_t motorId,
                                    uint8_t positionMSB,
                                    uint8_t positionMSB1,
                                    uint8_t positionMSB2,
                                    uint8_t positionLSB);
  void stepper_run_speed_to_position(uint8_t motorId);
  void stepper_stop(uint8_t motorId);
  void stepper_disable_outputs(uint8_t motorId);
  void stepper_enable_outputs(uint8_t motorId);
  void stepper_set_minimum_pulse_width(uint8_t motorId,
                                       uint8_t pulseWidthMSB,
                                       uint8_t pulseWidthLSB);
  void stepper_set_enable_pin(uint8_t motorId, uint8_t enablePin);
  void stepper_set_3_pins_inverted(uint8_t motorId,
                                   bool directionInvert,
                                   bool stepInvert,
                                   bool nableInvert);
  void stepper_set_4_pins_inverted(uint8_t motorId,
                                   bool pin1Invert,
                                   bool pin2Invert,
                                   bool pin3Invert,
                                   bool pin4Invert,
                                   bool enableInvert);
  void stepper_is_running(uint8_t motorId);

private:
  // Stepper motor data
  AccelStepper* steppers[MAX_NUMBER_OF_STEPPERS];

  // Stepper run modes
  uint8_t stepper_run_modes[MAX_NUMBER_OF_STEPPERS];
};
} // namespace OASIS

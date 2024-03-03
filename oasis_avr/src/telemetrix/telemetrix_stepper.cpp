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

#include "telemetrix_stepper.hpp"

#include "telemetrix_commands.hpp"
#include "telemetrix_reports.hpp"

#include <AccelStepper.h>
#include <HardwareSerial.h>

using namespace OASIS;

TelemetrixStepper::TelemetrixStepper()
{
  for (unsigned int i = 0; i < MAX_NUMBER_OF_STEPPERS; ++i)
    stepper_run_modes[i] = STEPPER_STOP;
}

void TelemetrixStepper::RunSteppers()
{
  for (unsigned int i = 0; i < MAX_NUMBER_OF_STEPPERS; ++i)
  {
    if (stepper_run_modes[i] == STEPPER_STOP)
    {
      continue;
    }
    else
    {
      steppers[i]->enableOutputs();

      switch (stepper_run_modes[i])
      {
        case STEPPER_RUN:
        {
          steppers[i]->run();

          const bool running = steppers[i]->isRunning();
          if (!running)
          {
            uint8_t report_message[3] = {2, STEPPER_RUN_COMPLETE_REPORT, static_cast<uint8_t>(i)};
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }

          break;
        }
        case STEPPER_RUN_SPEED:
        {
          steppers[i]->runSpeed();

          break;
        }
        case STEPPER_RUN_SPEED_TO_POSITION:
        {
          steppers[i]->runSpeedToPosition();

          const long target_position = steppers[i]->targetPosition();
          if (target_position == steppers[i]->currentPosition())
          {
            uint8_t report_message[3] = {2, STEPPER_RUN_COMPLETE_REPORT, static_cast<uint8_t>(i)};
            Serial.write(report_message, 3);
            stepper_run_modes[i] = STEPPER_STOP;
          }

          break;
        }
        default:
          break;
      }
    }
  }
}

void TelemetrixStepper::set_pin_mode_stepper(uint8_t motorId,
                                             uint8_t interface,
                                             uint8_t pin1,
                                             uint8_t pin2,
                                             uint8_t pin3,
                                             uint8_t pin4,
                                             bool enable)
{

  // Instantiate a stepper object and store it in the stepper array
  steppers[motorId] = new AccelStepper(interface, pin1, pin2, pin3, pin4, enable);
}

void TelemetrixStepper::stepper_move_to(uint8_t motorId,
                                        uint8_t positionMSB,
                                        uint8_t positionMSB1,
                                        uint8_t positionMSB2,
                                        uint8_t positionLSB,
                                        bool polarity)
{
  // Convert the 4 position bytes to a long
  long position = static_cast<long>(positionMSB) << 24;
  position += static_cast<long>(positionMSB1) << 16;
  position += positionMSB2 << 8;
  position += positionLSB;
  if (polarity)
    position *= -1;

  steppers[motorId]->moveTo(position);
}

void TelemetrixStepper::stepper_move(uint8_t motorId,
                                     uint8_t positionMSB,
                                     uint8_t positionMSB1,
                                     uint8_t positionMSB2,
                                     uint8_t positionLSB,
                                     bool polarity)
{
  // Convert the 4 position bytes to a long
  long position = static_cast<long>(positionMSB) << 24;
  position += static_cast<long>(positionMSB1) << 16;
  position += positionMSB2 << 8;
  position += positionLSB;
  if (polarity)
    position *= -1;

  steppers[motorId]->move(position);
}

void TelemetrixStepper::stepper_run(uint8_t motorId)
{
  stepper_run_modes[motorId] = STEPPER_RUN;
}

void TelemetrixStepper::stepper_run_speed(uint8_t motorId)
{
  stepper_run_modes[motorId] = STEPPER_RUN_SPEED;
}

void TelemetrixStepper::stepper_set_max_speed(uint8_t motorId, uint8_t speedMSB, uint8_t speedLSB)
{
  const float max_speed = static_cast<float>((speedMSB << 8) + speedLSB);

  steppers[motorId]->setMaxSpeed(max_speed);
}

void TelemetrixStepper::stepper_set_acceleration(uint8_t motorId,
                                                 uint8_t accelMSB,
                                                 uint8_t accelLSB)
{
  const float acceleration = static_cast<float>((accelMSB << 8) + accelLSB);

  steppers[motorId]->setAcceleration(acceleration);
}

void TelemetrixStepper::stepper_set_speed(uint8_t motorId, uint8_t speedMSB, uint8_t speedLSB)
{
  const float speed = static_cast<float>((speedMSB << 8) + speedLSB);

  steppers[motorId]->setSpeed(speed);
}

void TelemetrixStepper::stepper_get_distance_to_go(uint8_t motorId)
{
  // report = STEPPER_DISTANCE_TO_GO, motor_id, distance(8 bytes)

  uint8_t report_message[7] = {6, STEPPER_DISTANCE_TO_GO, motorId};

  const long distanceToGo = steppers[motorId]->distanceToGo();

  report_message[3] = static_cast<uint8_t>((distanceToGo & 0xFF000000) >> 24);
  report_message[4] = static_cast<uint8_t>((distanceToGo & 0x00FF0000) >> 16);
  report_message[5] = static_cast<uint8_t>((distanceToGo & 0x0000FF00) >> 8);
  report_message[6] = static_cast<uint8_t>((distanceToGo & 0x000000FF));

  Serial.write(report_message, 7);
}

void TelemetrixStepper::stepper_get_target_position(uint8_t motorId)
{
  // report = STEPPER_TARGET_POSITION, motor_id, distance(8 bytes)

  uint8_t report_message[7] = {6, STEPPER_TARGET_POSITION, motorId};

  const long target = steppers[motorId]->targetPosition();

  report_message[3] = static_cast<uint8_t>((target & 0xFF000000) >> 24);
  report_message[4] = static_cast<uint8_t>((target & 0x00FF0000) >> 16);
  report_message[5] = static_cast<uint8_t>((target & 0x0000FF00) >> 8);
  report_message[6] = static_cast<uint8_t>((target & 0x000000FF));

  Serial.write(report_message, 7);
}

void TelemetrixStepper::stepper_get_current_position(uint8_t motorId)
{
  // report = STEPPER_CURRENT_POSITION, motor_id, distance(8 bytes)

  uint8_t report_message[7] = {6, STEPPER_CURRENT_POSITION, motorId};

  long position = steppers[motorId]->targetPosition();

  report_message[3] = static_cast<uint8_t>((position & 0xFF000000) >> 24);
  report_message[4] = static_cast<uint8_t>((position & 0x00FF0000) >> 16);
  report_message[5] = static_cast<uint8_t>((position & 0x0000FF00) >> 8);
  report_message[6] = static_cast<uint8_t>((position & 0x000000FF));

  Serial.write(report_message, 7);
}

void TelemetrixStepper::stepper_set_current_position(uint8_t motorId,
                                                     uint8_t positionMSB,
                                                     uint8_t positionMSB1,
                                                     uint8_t positionMSB2,
                                                     uint8_t positionLSB)
{
  // Convert the 4 position bytes to a long
  long position = static_cast<long>(positionMSB) << 24;
  position += static_cast<long>(positionMSB1) << 16;
  position += positionMSB2 << 8;
  position += positionLSB;

  steppers[motorId]->setCurrentPosition(position);
}

void TelemetrixStepper::stepper_run_speed_to_position(uint8_t motorId)
{
  stepper_run_modes[motorId] = STEPPER_RUN_SPEED_TO_POSITION;
}

void TelemetrixStepper::stepper_stop(uint8_t motorId)
{
  steppers[motorId]->stop();
  steppers[motorId]->disableOutputs();
  stepper_run_modes[motorId] = STEPPER_STOP;
}

void TelemetrixStepper::stepper_disable_outputs(uint8_t motorId)
{
  steppers[motorId]->disableOutputs();
}

void TelemetrixStepper::stepper_enable_outputs(uint8_t motorId)
{
  steppers[motorId]->enableOutputs();
}

void TelemetrixStepper::stepper_set_minimum_pulse_width(uint8_t motorId,
                                                        uint8_t pulseWidthMSB,
                                                        uint8_t pulseWidthLSB)
{
  const unsigned int pulse_width = (pulseWidthMSB << 8) + pulseWidthLSB;

  steppers[motorId]->setMinPulseWidth(pulse_width);
}

void TelemetrixStepper::stepper_set_enable_pin(uint8_t motorId, uint8_t enablePin)
{
  steppers[motorId]->setEnablePin(enablePin);
}

void TelemetrixStepper::stepper_set_3_pins_inverted(uint8_t motorId,
                                                    bool directionInvert,
                                                    bool stepInvert,
                                                    bool enableInvert)
{
  steppers[motorId]->setPinsInverted(directionInvert, stepInvert, enableInvert);
}

void TelemetrixStepper::stepper_set_4_pins_inverted(uint8_t motorId,
                                                    bool pin1Invert,
                                                    bool pin2Invert,
                                                    bool pin3Invert,
                                                    bool pin4Invert,
                                                    bool enableInvert)
{
  steppers[motorId]->setPinsInverted(pin1Invert, pin2Invert, pin3Invert, pin4Invert, enableInvert);
}

void TelemetrixStepper::stepper_is_running(uint8_t motorId)
{
  // report = STEPPER_IS_RUNNING, motor_id, distance(8 bytes)

  uint8_t report_message[3] = {2, STEPPER_RUNNING_REPORT, motorId};

  report_message[2] = steppers[motorId]->isRunning();

  Serial.write(report_message, 3);
}

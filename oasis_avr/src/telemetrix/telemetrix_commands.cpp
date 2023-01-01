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

#include "telemetrix_commands.hpp"

#include "telemetrix_dht.hpp"
#include "telemetrix_i2c.hpp"
#include "telemetrix_memory.hpp"
#include "telemetrix_one_wire.hpp"
#include "telemetrix_pins.hpp"
#include "telemetrix_reports.hpp"
#include "telemetrix_server.hpp"
#include "telemetrix_servo.hpp"
#include "telemetrix_sonar.hpp"
#include "telemetrix_spi.hpp"
#include "telemetrix_stepper.hpp"
#include "telemetrix_version.hpp"

#include <string.h>

#include <Arduino.h>
#include <HardwareSerial.h>

////////////////////////////////////////////////////////////////////////////////
// Arduino ID
////////////////////////////////////////////////////////////////////////////////

// This value must be the same as specified when instantiating the
// telemetrix client. The client defaults to a value of 1.
// This value is used for the client to auto-discover and to
// connect to a specific board regardless of the current com port
// it is currently connected to.

#define ARDUINO_ID 1

////////////////////////////////////////////////////////////////////////////////
// Client Command Related Defines and Support
////////////////////////////////////////////////////////////////////////////////

using namespace OASIS;

namespace
{
// The command_func is a pointer the command's function.
struct command_descriptor
{
  // A pointer to the command processing function
  void (*command_func)(void);
};

// An array of pointers to the command functions.
// The list must be in the same order as the command defines.
static const command_descriptor commandTable[] = {
    {&TelemetrixCommands::serial_loopback},
    {&TelemetrixCommands::set_pin_mode},
    {&TelemetrixCommands::digital_write},
    {&TelemetrixCommands::analog_write},
    {&TelemetrixCommands::modify_reporting},
    {&TelemetrixCommands::get_firmware_version},
    {&TelemetrixCommands::are_you_there},
    {&TelemetrixCommands::servo_attach},
    {&TelemetrixCommands::servo_write},
    {&TelemetrixCommands::servo_detach},
    {&TelemetrixCommands::i2c_begin},
    {&TelemetrixCommands::i2c_read},
    {&TelemetrixCommands::i2c_write},
    {&TelemetrixCommands::sonar_new},
    {&TelemetrixCommands::dht_new},
    {&TelemetrixCommands::stop_all_reports},
    {&TelemetrixCommands::set_analog_scanning_interval},
    {&TelemetrixCommands::enable_all_reports},
    {&TelemetrixCommands::reset_data},
    {&TelemetrixCommands::init_spi},
    {&TelemetrixCommands::write_blocking_spi},
    {&TelemetrixCommands::read_blocking_spi},
    {&TelemetrixCommands::set_format_spi},
    {&TelemetrixCommands::spi_cs_control},
    {&TelemetrixCommands::onewire_init},
    {&TelemetrixCommands::onewire_reset},
    {&TelemetrixCommands::onewire_select},
    {&TelemetrixCommands::onewire_skip},
    {&TelemetrixCommands::onewire_write},
    {&TelemetrixCommands::onewire_read},
    {&TelemetrixCommands::onewire_reset_search},
    {&TelemetrixCommands::onewire_search},
    {&TelemetrixCommands::onewire_crc8},
    {&TelemetrixCommands::set_pin_mode_stepper},
    {&TelemetrixCommands::stepper_move_to},
    {&TelemetrixCommands::stepper_move},
    {&TelemetrixCommands::stepper_run},
    {&TelemetrixCommands::stepper_run_speed},
    {&TelemetrixCommands::stepper_set_max_speed},
    {&TelemetrixCommands::stepper_set_acceleration},
    {&TelemetrixCommands::stepper_set_speed},
    (&TelemetrixCommands::stepper_set_current_position),
    (&TelemetrixCommands::stepper_run_speed_to_position),
    (&TelemetrixCommands::stepper_stop),
    (&TelemetrixCommands::stepper_disable_outputs),
    (&TelemetrixCommands::stepper_enable_outputs),
    (&TelemetrixCommands::stepper_set_minimum_pulse_width),
    (&TelemetrixCommands::stepper_set_enable_pin),
    (&TelemetrixCommands::stepper_set_3_pins_inverted),
    (&TelemetrixCommands::stepper_set_4_pins_inverted),
    (&TelemetrixCommands::stepper_is_running),
    (&TelemetrixCommands::stepper_get_current_position),
    {&TelemetrixCommands::stepper_get_distance_to_go},
    (&TelemetrixCommands::stepper_get_target_position),
    (&TelemetrixCommands::get_features),
    (&TelemetrixCommands::set_memory_reporting_interval),
};

} // namespace

////////////////////////////////////////////////////////////////////////////////
// Telemetrix command handlers
////////////////////////////////////////////////////////////////////////////////

TelemetrixServer* TelemetrixCommands::m_server{nullptr};
uint8_t TelemetrixCommands::commandBuffer[MAX_COMMAND_LENGTH]{};

void TelemetrixCommands::RegisterServer(TelemetrixServer* server)
{
  m_server = server;
}

void TelemetrixCommands::GetNextCommand()
{
  // Clear the command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));

  // If there is no command waiting, then return
  if (!Serial.available())
    return;

  // Get the packet length
  const uint8_t packetLength = static_cast<uint8_t>(Serial.read());

  while (!Serial.available())
    yield();

  // Get the command byte
  const uint8_t command = static_cast<uint8_t>(Serial.read());
  const command_descriptor commandEntry = commandTable[command];

  if (packetLength > 1)
  {
    // Get the data for that command
    for (unsigned int i = 0; i < packetLength - 1; ++i)
    {
      // Need this delay or data read is not correct
      while (!Serial.available())
        yield();

      commandBuffer[i] = static_cast<uint8_t>(Serial.read());
    }
  }
  commandEntry.command_func();
}

void TelemetrixCommands::serial_loopback()
{
  const uint8_t loop_back_buffer[3] = {2, static_cast<uint8_t>(SERIAL_LOOP_BACK), commandBuffer[0]};

  Serial.write(loop_back_buffer, 3);
}

void TelemetrixCommands::set_pin_mode()
{
  const uint8_t pin = commandBuffer[0];
  const uint8_t mode = commandBuffer[1];

  TelemetrixPins* pins = m_server->GetPins();

  switch (mode)
  {
    case INPUT:
      pins->set_pin_mode_input(pin, commandBuffer[2]);
      break;
    case INPUT_PULLUP:
      pins->set_pin_mode_input_pullup(pin, commandBuffer[2]);
      break;
    case OUTPUT:
      pins->set_pin_mode_output(pin);
      break;
    case AT_ANALOG:
      pins->set_pin_mode_analog(pin, (commandBuffer[2] << 8) + commandBuffer[3], commandBuffer[4]);
      break;
    default:
      break;
  }
}

void TelemetrixCommands::digital_write()
{
  const uint8_t pin = commandBuffer[0];
  const uint8_t value = commandBuffer[1];

  TelemetrixPins* pins = m_server->GetPins();
  pins->DigitalWrite(pin, value);
}

void TelemetrixCommands::analog_write()
{
  const uint8_t pin = commandBuffer[0];
  const unsigned int value = (commandBuffer[1] << 8) + commandBuffer[2];

  TelemetrixPins* pins = m_server->GetPins();
  pins->AnalogWrite(pin, value);
}

void TelemetrixCommands::modify_reporting()
{
  const uint8_t pin = commandBuffer[1];
  const uint8_t reporting = commandBuffer[0];

  TelemetrixPins* pins = m_server->GetPins();
  pins->modify_reporting(pin, reporting);
}

void TelemetrixCommands::get_firmware_version()
{
  const uint8_t report_message[5] = {4, FIRMWARE_REPORT, FIRMWARE_MAJOR, FIRMWARE_MINOR,
                                     FIRMWARE_PATCH};

  Serial.write(report_message, 5);
}

void TelemetrixCommands::are_you_there()
{
  const uint8_t report_message[3] = {2, I_AM_HERE, ARDUINO_ID};

  Serial.write(report_message, 3);
}

void TelemetrixCommands::servo_attach()
{
#if defined(ENABLE_SERVO)
  const uint8_t pin = commandBuffer[0];
  const int minpulse = (commandBuffer[1] << 8) + commandBuffer[2];
  const int maxpulse = (commandBuffer[3] << 8) + commandBuffer[4];

  TelemetrixServo* servo = m_server->GetServo();
  servo->ServoAttach(pin, minpulse, maxpulse);
#endif
}

void TelemetrixCommands::servo_write()
{
#if defined(ENABLE_SERVO)
  const uint8_t pin = commandBuffer[0];
  const int angle = commandBuffer[1];

  TelemetrixServo* servo = m_server->GetServo();
  servo->ServoWrite(pin, angle);
#endif
}

void TelemetrixCommands::servo_detach()
{
#if defined(ENABLE_SERVO)
  TelemetrixServo* servo = m_server->GetServo();
  servo->ServoDetach(commandBuffer[0]);
#endif
}

void TelemetrixCommands::i2c_begin()
{
#if defined(ENABLE_I2C)
  const uint8_t i2cPort = commandBuffer[0];

  TelemetrixI2C* i2c = m_server->GetI2C();
  i2c->i2c_begin(i2cPort);
#endif
}

void TelemetrixCommands::i2c_read()
{
#if defined(ENABLE_I2C)
  const uint8_t address = commandBuffer[0];
  const uint8_t theRegister = commandBuffer[1];
  const uint8_t byteCount = commandBuffer[2];
  const bool stopTransmitting = (commandBuffer[3] != 0);
  const uint8_t i2cPort = commandBuffer[4];
  const bool writeFlag = (commandBuffer[5] != 0);

  TelemetrixI2C* i2c = m_server->GetI2C();
  i2c->i2c_read(address, theRegister, byteCount, stopTransmitting, i2cPort, writeFlag);
#endif
}

void TelemetrixCommands::i2c_write()
{
#if defined(ENABLE_I2C)
  const uint8_t byteCount = commandBuffer[0];
  const uint8_t deviceAddress = commandBuffer[1];
  const uint8_t i2cPort = commandBuffer[2];
  const uint8_t* data = commandBuffer + 3;

  TelemetrixI2C* i2c = m_server->GetI2C();
  i2c->i2c_write(byteCount, deviceAddress, i2cPort, data);
#endif
}

void TelemetrixCommands::sonar_new()
{
#if defined(ENABLE_SONAR)
  const uint8_t triggerPin = commandBuffer[0];
  const uint8_t echoPin = commandBuffer[1];

  TelemetrixSonar* sonar = m_server->GetSonar();
  sonar->AttachSonar(triggerPin, echoPin);
#endif
}

void TelemetrixCommands::dht_new()
{
#if defined(ENABLE_DHT)
  TelemetrixDHT* dht = m_server->GetDHT();
  dht->dht_new(commandBuffer[0], commandBuffer[1]);
#endif
}

void TelemetrixCommands::stop_all_reports()
{
  m_server->SetReporting(false);
  delay(20);
  Serial.flush();
}

void TelemetrixCommands::set_analog_scanning_interval()
{
  const uint8_t analogSamplingInterval = commandBuffer[0];

  TelemetrixPins* pins = m_server->GetPins();
  pins->set_analog_sampling_interval(analogSamplingInterval);
}

void TelemetrixCommands::enable_all_reports()
{
  Serial.flush();
  m_server->SetReporting(true);
  delay(20);
}

void TelemetrixCommands::reset_data()
{
  // Fist stop all reporting
  stop_all_reports();

  TelemetrixMemory* memory = m_server->GetMemory();
  memory->ResetData();

  TelemetrixPins* pins = m_server->GetPins();
  pins->reset_data();

#if defined(ENABLE_SERVO)
  TelemetrixServo* servo = m_server->GetServo();
  servo->ResetData();
#endif

#if defined(ENABLE_SONAR)
  TelemetrixSonar* sonar = m_server->GetSonar();
  sonar->ResetData();
#endif

#if defined(ENABLE_DHT)
  TelemetrixDHT* dht = m_server->GetDHT();
  dht->reset_data();
#endif

  enable_all_reports();
}

void TelemetrixCommands::init_pin_structures()
{
  TelemetrixPins* pins = m_server->GetPins();
  pins->InitPinStructures();
}

void TelemetrixCommands::init_spi()
{
#if defined(ENABLE_SPI)
  uint8_t pinCount = commandBuffer[0];
  const uint8_t* pins = commandBuffer + 1;

  TelemetrixSPI* spi = m_server->GetSPI();
  spi->init_spi(pinCount, pins);
#endif
}

void TelemetrixCommands::write_blocking_spi()
{
#if defined(ENABLE_SPI)
  uint8_t byteCount = commandBuffer[0];
  const uint8_t* data = commandBuffer + 1;

  TelemetrixSPI* spi = m_server->GetSPI();
  spi->write_blocking_spi(byteCount, data);
#endif
}

void TelemetrixCommands::read_blocking_spi()
{
#if defined(ENABLE_SPI)
  const uint8_t byteCount = commandBuffer[0]; // Number of bytes to read
  const uint8_t readRegister = commandBuffer[1];

  TelemetrixSPI* spi = m_server->GetSPI();
  spi->read_blocking_spi(byteCount, readRegister);
#endif
}

void TelemetrixCommands::set_format_spi()
{
#if defined(ENABLE_SPI)
  const uint32_t clock = commandBuffer[0];
  const uint8_t bitOrder = commandBuffer[1];
  const uint8_t dataMode = (commandBuffer[2] != 0 ? MSBFIRST : LSBFIRST);

  TelemetrixSPI* spi = m_server->GetSPI();
  spi->set_format_spi(clock, bitOrder, dataMode);
#endif
}

void TelemetrixCommands::spi_cs_control()
{
#if defined(ENABLE_SPI)
  const uint8_t csPin = commandBuffer[0];
  const uint8_t csState = commandBuffer[1];

  TelemetrixSPI* spi = m_server->GetSPI();
  spi->spi_cs_control(csPin, csState);
#endif
}

void TelemetrixCommands::onewire_init()
{
#if defined(ENABLE_ONE_WIRE)
  const uint8_t pin = commandBuffer[0];

  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_init(pin);
#endif
}


void TelemetrixCommands::onewire_reset()
{
#if defined(ENABLE_ONE_WIRE)
  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_reset();
#endif
}

void TelemetrixCommands::onewire_select()
{
#if defined(ENABLE_ONE_WIRE)
  uint8_t deviceAddress[8];

  for (int i = 0; i < 8; i++)
    deviceAddress[i] = commandBuffer[i];

  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_select(deviceAddress);
#endif
}

void TelemetrixCommands::onewire_skip()
{
#if defined(ENABLE_ONE_WIRE)
  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_skip();
#endif
}

void TelemetrixCommands::onewire_write()
{
#if defined(ENABLE_ONE_WIRE)
  const uint8_t value = commandBuffer[0];
  const bool power = (commandBuffer[1] != 0);

  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_write(value, power);
#endif
}

void TelemetrixCommands::onewire_read()
{
#if defined(ENABLE_ONE_WIRE)
  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_read();
#endif
}

void TelemetrixCommands::onewire_reset_search()
{
#if defined(ENABLE_ONE_WIRE)
  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_reset_search();
#endif
}

void TelemetrixCommands::onewire_search()
{
#if defined(ENABLE_ONE_WIRE)
  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_search();
#endif
}

void TelemetrixCommands::onewire_crc8()
{
#if defined(ENABLE_ONE_WIRE)
  const uint8_t length = commandBuffer[0];
  const uint8_t* address = commandBuffer + 1;

  TelemetrixOneWire* oneWire = m_server->GetOneWire();
  oneWire->onewire_crc8(address, length);
#endif
}

void TelemetrixCommands::set_pin_mode_stepper()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t interface = commandBuffer[1];
  const uint8_t pin1 = commandBuffer[2];
  const uint8_t pin2 = commandBuffer[3];
  const uint8_t pin3 = commandBuffer[4];
  const uint8_t pin4 = commandBuffer[5];
  const bool enable = (commandBuffer[6] != 0);

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->set_pin_mode_stepper(motorId, interface, pin1, pin2, pin3, pin4, enable);
#endif
}

void TelemetrixCommands::stepper_move_to()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t positionMSB = commandBuffer[1];
  const uint8_t positionMSB1 = commandBuffer[2];
  const uint8_t positionMSB2 = commandBuffer[3];
  const uint8_t positionLSB = commandBuffer[4];
  const bool polarity = (commandBuffer[5] != 0);

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_move_to(motorId, positionMSB, positionMSB1, positionMSB2, positionLSB, polarity);
#endif
}

void TelemetrixCommands::stepper_move()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t positionMSB = commandBuffer[1];
  const uint8_t positionMSB1 = commandBuffer[2];
  const uint8_t positionMSB2 = commandBuffer[3];
  const uint8_t positionLSB = commandBuffer[4];
  const bool polarity = (commandBuffer[5] != 0);

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_move(motorId, positionMSB, positionMSB1, positionMSB2, positionLSB, polarity);
#endif
}

void TelemetrixCommands::stepper_run()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_run(motorId);
#endif
}

void TelemetrixCommands::stepper_run_speed()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_run_speed(motorId);
#endif
}

void TelemetrixCommands::stepper_set_max_speed()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t speedMSB = commandBuffer[1];
  const uint8_t speedLSB = commandBuffer[2];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_max_speed(motorId, speedMSB, speedLSB);
#endif
}

void TelemetrixCommands::stepper_set_acceleration()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t accelMSB = commandBuffer[1];
  const uint8_t accelLSB = commandBuffer[2];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_acceleration(motorId, accelMSB, accelLSB);
#endif
}

void TelemetrixCommands::stepper_set_speed()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t speedMSB = commandBuffer[1];
  const uint8_t speedLSB = commandBuffer[2];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_speed(motorId, speedMSB, speedLSB);
#endif
}

void TelemetrixCommands::stepper_get_distance_to_go()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_get_distance_to_go(motorId);
#endif
}

void TelemetrixCommands::stepper_get_target_position()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_get_target_position(motorId);
#endif
}

void TelemetrixCommands::stepper_get_current_position()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_get_current_position(motorId);
#endif
}

void TelemetrixCommands::stepper_set_current_position()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t positionMSB = commandBuffer[1];
  const uint8_t positionMSB1 = commandBuffer[2];
  const uint8_t positionMSB2 = commandBuffer[3];
  const uint8_t positionLSB = commandBuffer[4];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_current_position(motorId, positionMSB, positionMSB1, positionMSB2,
                                        positionLSB);
#endif
}

void TelemetrixCommands::stepper_run_speed_to_position()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_run_speed_to_position(motorId);
#endif
}

void TelemetrixCommands::stepper_stop()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_stop(motorId);
#endif
}

void TelemetrixCommands::stepper_disable_outputs()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_disable_outputs(motorId);
#endif
}

void TelemetrixCommands::stepper_enable_outputs()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_enable_outputs(motorId);
#endif
}

void TelemetrixCommands::stepper_set_minimum_pulse_width()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t pulseWidthMSB = commandBuffer[1];
  const uint8_t pulseWidthLSB = commandBuffer[2];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_minimum_pulse_width(motorId, pulseWidthMSB, pulseWidthLSB);
#endif
}

void TelemetrixCommands::stepper_set_enable_pin()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const uint8_t enablePin = commandBuffer[1];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_enable_pin(motorId, enablePin);
#endif
}

void TelemetrixCommands::stepper_set_3_pins_inverted()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const bool directionInvert = commandBuffer[1];
  const bool stepInvert = commandBuffer[2];
  const bool enableInvert = commandBuffer[3];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_3_pins_inverted(motorId, directionInvert, stepInvert, enableInvert);
#endif
}

void TelemetrixCommands::stepper_set_4_pins_inverted()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];
  const bool pin1 = commandBuffer[1];
  const bool pin2 = commandBuffer[2];
  const bool pin3 = commandBuffer[3];
  const bool pin4 = commandBuffer[4];
  const bool enable = commandBuffer[5];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_set_4_pins_inverted(motorId, pin1, pin2, pin3, pin4, enable);
#endif
}

void TelemetrixCommands::stepper_is_running()
{
#if defined(ENABLE_STEPPER)
  const uint8_t motorId = commandBuffer[0];

  TelemetrixStepper* stepper = m_server->GetStepper();
  stepper->stepper_is_running(motorId);
#endif
}

void TelemetrixCommands::get_features()
{
  uint8_t report_message[3] = {2, FEATURES, m_server->Features()};
  Serial.write(report_message, 3);
}

void TelemetrixCommands::set_memory_reporting_interval()
{
  const uint32_t intervalMs = (commandBuffer[0] << 24) + (commandBuffer[1] << 16) +
                              (commandBuffer[2] << 8) + commandBuffer[3];

  TelemetrixMemory* memory = m_server->GetMemory();
  memory->SetReportingInterval(intervalMs);
}

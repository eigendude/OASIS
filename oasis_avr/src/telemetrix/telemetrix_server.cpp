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

#include "telemetrix_server.hpp"

#include "telemetrix_commands.hpp"
#include "telemetrix_dht.hpp"
#include "telemetrix_features.hpp"
#include "telemetrix_i2c.hpp"
#include "telemetrix_memory.hpp"
#include "telemetrix_one_wire.hpp"
#include "telemetrix_pins.hpp"
#include "telemetrix_servo.hpp"
#include "telemetrix_sonar.hpp"
#include "telemetrix_spi.hpp"
#include "telemetrix_stepper.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixServer::Setup()
{
  Serial.begin(115200);

  static TelemetrixPins pins;
  m_pins = &pins;

  static TelemetrixMemory memory;
  m_memory = &memory;

  // Set up features for enabled features
#if defined(ENABLE_DHT)
  static TelemetrixDHT dht;
  m_dht = &dht;
  m_features |= DHT_FEATURE;
#endif

#if defined(ENABLE_I2C)
  static TelemetrixI2C i2c;
  m_i2c = &i2c;
  m_features |= I2C_FEATURE;
#endif

#if defined(ENABLE_ONE_WIRE)
  static TelemetrixOneWire oneWire;
  m_oneWire = &oneWire;
  m_features |= ONEWIRE_FEATURE;
#endif

#if defined(ENABLE_SERVO)
  static TelemetrixServo servo;
  m_servo = &servo;
  m_features |= SERVO_FEATURE;
#endif

#if defined(ENABLE_SONAR)
  static TelemetrixSonar sonar;
  m_sonar = &sonar;
  m_features |= SONAR_FEATURE;
#endif

#if defined(ENABLE_SPI)
  static TelemetrixSPI spi;
  m_spi = &spi;
  m_features |= SPI_FEATURE;
#endif

#if defined(ENABLE_STEPPER)
  static TelemetrixStepper stepper;
  m_stepper = &stepper;
  m_features |= STEPPERS_FEATURE;
#endif

  TelemetrixCommands::RegisterServer(this);
}

void TelemetrixServer::Loop()
{
  // Keep processing incoming commands
  TelemetrixCommands::GetNextCommand();

  if (!m_stopReports)
  {
    ScanDigitalInputs();
    ScanAnalogInputs();
    ScanMemory();
    ScanSonars();
    ScanDHTs();
    RunSteppers();
  }
}

void TelemetrixServer::ScanDigitalInputs()
{
  m_pins->scan_digital_inputs();
}

void TelemetrixServer::ScanAnalogInputs()
{
  m_pins->scan_analog_inputs();
}

void TelemetrixServer::ScanMemory()
{
  m_memory->ScanMemory();
}

void TelemetrixServer::ScanSonars()
{
#if defined(ENABLE_SONAR)
  m_sonar->ScanSonars();
#endif
}

void TelemetrixServer::ScanDHTs()
{
#if defined(ENABLE_DHT)
  m_dht->scan_dhts();
#endif
}

void TelemetrixServer::RunSteppers()
{
#if defined(ENABLE_STEPPER)
  m_stepper->RunSteppers();
#endif
}

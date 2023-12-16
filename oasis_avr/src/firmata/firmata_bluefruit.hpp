/*
 *  Copyright (C) 2021-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Adafruit_BluefruitLE_nRF51 under the
 *  BSD 3-Clause License.
 *  Copyright (C) 2016, Adafruit Industries (adafruit.com)
 *
 *  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include "firmata_subsystem.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

#include <Adafruit_BluefruitLE_SPI.h>

namespace OASIS
{

/*!
 * \brief Bluefruit subsystem for Firmata
 *
 * This class uses the Bluefruit callback API for specific events:
 *
 *   - setConnectCallback()
 *   - setDisconnectCallback()
 *   - setBleUartRxCallback()
 *   - setBleGattRxCallback()
 *
 * Furthermore, update() must be called inside loop() for callback to be
 * executed.
 *
 * The subsystem will add an custom service with 2 writable characteristics,
 * and install callback to execute when there is an update from central device.
 *
 *   - one hold string
 *   - one hold a 4-byte integer
 */
class FirmataBluefruit : public FirmataSubsystem
{
public:
  FirmataBluefruit();

  // Implementation of FirmataSubsystem
  void Setup() override;
  void Loop() override;

private:
  // BLE callbacks (instance)
  void OnConnect();
  void OnDisconnect();
  void OnUartRx(char data[], uint16_t len);

  // BLE callbacks (static)
  static void StaticOnConnect();
  static void StaticOnDisconnect();
  static void StaticOnUartRx(char data[], uint16_t length);

  // Reference used in static callbacks
  static FirmataBluefruit* m_instance;

  // The Bluefruit object is created with hardware SPI, using SCK/MOSI/MISO
  // hardware SPI pins and then user selected CS/IRQ/RST defined in the cpp
  // file
  Adafruit_BluefruitLE_SPI m_ble;

  // State parameters
  bool m_started{false};
  bool m_connected{false};
};

} // namespace OASIS

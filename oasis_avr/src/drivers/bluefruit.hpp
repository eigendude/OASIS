/*
 *  Copyright (C) 2021 Garrett Brown
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

#include <stdint.h>

#include <Adafruit_BluefruitLE_SPI.h>

namespace OASIS
{

/*!
 * \brief Bluefruit subsystem
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
 * The subsystem will add a custom service with 2 writable characteristics,
 * and install a callback to execute when there is an update from the central
 * device:
 *
 *   - One holds string
 *   - One holds a 4-byte integer
 */
class Bluefruit
{
public:
  Bluefruit();

  // Lifecycle functions
  void Setup();
  void Loop();

  // Bluetooth functions
  bool WriteUart(const uint8_t* buffer, size_t size);
  bool ReadUart(uint8_t& data);

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
  static Bluefruit* m_instance;

  // The Bluefruit object is created with hardware SPI, using SCK/MOSI/MISO
  // hardware SPI pins and then user selected CS/IRQ/RST defined in the cpp
  // file
  Adafruit_BluefruitLE_SPI m_ble;

  // State parameters
  bool m_connected{false};
};

} // namespace OASIS

/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Adafruit_BluefruitLE_nRF51 under the
 *  BSD 3-Clause License.
 *  Copyright (C) 2016, Adafruit Industries (adafruit.com)
 *
 *  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 *  See DOCS/LICENSING.md for more information.
 */

#include "bluefruit.hpp"

#include <string.h>

#include <Arduino.h>

using namespace OASIS;

namespace OASIS
{

// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
//constexpr uint8_t BUFSIZE = 160; // Size of the read buffer for incoming data

// Period in milliseconds between each Bluetooth event scan. See Adafruit_BLE.h
constexpr uint32_t BLUETOOTH_SCAN_INTERVAL_MS = 200;

// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------

constexpr uint8_t BLUEFRUIT_SPI_CS = 8;
constexpr uint8_t BLUEFRUIT_SPI_IRQ = 7;
constexpr uint8_t BLUEFRUIT_SPI_RST = 4; // Optional but recommended, set to -1 if unused

} // namespace OASIS

Bluefruit* Bluefruit::m_instance{nullptr};

Bluefruit::Bluefruit() : m_ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST)
{
}

void Bluefruit::Setup()
{
  // Initialize reference for static callbacks
  m_instance = this;

  // Initialize Bluetooth
  if (!m_ble.begin(false, true))
    return;

  const bool FACTORYRESET_ENABLE = true; // TODO
  if (FACTORYRESET_ENABLE)
  {
    // Perform a factory reset to make sure everything is in a known state
    m_ble.factoryReset();
  }

  // Disable command echo from Bluefruit
  m_ble.echo(false);

  // Install callbacks
  m_ble.setConnectCallback(StaticOnConnect);
  m_ble.setDisconnectCallback(StaticOnDisconnect);
  m_ble.setBleUartRxCallback(StaticOnUartRx);

  // LED Activity command is only supported from 0.6.6
  if (m_ble.isVersionAtLeast("0.6.6"))
  {
    // Change Mode LED Activity, valid options are "DISABLE", "MODE", "BLEUART",
    // "HWUART", "SPI" or "MANUAL"
    m_ble.sendCommandCheckOK("AT+HWModeLED=BLEUART");
  }

  // Set module to DATA mode to have a better throughput
  m_ble.setMode(BLUEFRUIT_MODE_DATA);
}

void Bluefruit::Loop()
{
  const unsigned long now = millis();

  const bool hasPendingEvent = m_ble.available() > 0;

  // Avoid running the full event query unless either the module signals that
  // work is pending (via the IRQ line exposed through available()) or the
  // periodic scan interval has elapsed. This keeps the Bluefruit handling
  // responsive without stalling other tasks like the heartbeat scheduler.
  if (!hasPendingEvent && static_cast<long>(now - m_nextScanMs) < 0)
    return;

  m_ble.update(0);

  m_nextScanMs = now + BLUETOOTH_SCAN_INTERVAL_MS;
}

bool Bluefruit::WriteUart(const uint8_t* buffer, size_t size)
{
  if (m_connected)
  {
    // Send the data
    return m_ble.writeBLEUart(buffer, size) == size;
  }

  return false;
}

bool Bluefruit::ReadUart(uint8_t& data)
{
  if (m_connected)
  {
    // Read a byte
    return m_ble.readBLEUart(&data, 1) == 1;
  }

  return false;
}

void Bluefruit::OnConnect()
{
  m_connected = true;
}

void Bluefruit::OnDisconnect()
{
  m_connected = false;
}

void Bluefruit::OnUartRx(char data[], uint16_t length)
{
  //Firmata.sendString("Bluetooth UART data received");
}

void Bluefruit::StaticOnConnect()
{
  if (m_instance != nullptr)
    m_instance->OnConnect();
}

void Bluefruit::StaticOnDisconnect()
{
  if (m_instance != nullptr)
    m_instance->OnDisconnect();
}

void Bluefruit::StaticOnUartRx(char data[], uint16_t length)
{
  if (m_instance != nullptr)
    m_instance->OnUartRx(data, length);
}

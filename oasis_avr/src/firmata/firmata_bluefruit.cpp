/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Adafruit_BluefruitLE_nRF51 under the
 *  BSD 3-Clause License.
 *  Copyright (C) 2016, Adafruit Industries (adafruit.com)
 *
 *  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_bluefruit.hpp"

#include <string.h>

//#include <FirmataExpress.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
// These settings are used in both SW UART, HW UART and SPI mode
// ----------------------------------------------------------------------------------------------
//constexpr uint8_t BUFSIZE = 160; // Size of the read buffer for incoming data

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

FirmataBluefruit* FirmataBluefruit::m_instance{nullptr};

FirmataBluefruit::FirmataBluefruit() : m_ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST)
{
}

void FirmataBluefruit::Setup()
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

  /*
  // Wait for connection to finish
  while (!m_ble.isConnected())
  {
    delay(5000);
  }

  // Wait for the connection to complete
  delay(1000);

  // Send some data
  uint8_t buffer[1]{};
  m_ble.writeBLEUart(buffer, sizeof(buffer));
  */

  m_started = true;
}

void FirmataBluefruit::Loop()
{
  if (!m_started)
    return;

  m_ble.update();
}

void FirmataBluefruit::OnConnect()
{
  m_connected = true;
  //Firmata.sendString("Bluetooth connected");
}

void FirmataBluefruit::OnDisconnect()
{
  m_connected = false;
  //Firmata.sendString("Bluetooth disconnected");
}

void FirmataBluefruit::OnUartRx(char data[], uint16_t length)
{
  //Firmata.sendString("Bluetooth UART data received");
}

void FirmataBluefruit::StaticOnConnect()
{
  if (m_instance != nullptr)
    m_instance->OnConnect();
}

void FirmataBluefruit::StaticOnDisconnect()
{
  if (m_instance != nullptr)
    m_instance->OnDisconnect();
}

void FirmataBluefruit::StaticOnUartRx(char data[], uint16_t length)
{
  if (m_instance != nullptr)
    m_instance->OnUartRx(data, length);
}

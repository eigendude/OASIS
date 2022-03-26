/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from ConfugurableFirmata under the LGPL 2.1 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2010-2011 Paul Stoffregen. All rights reserved.
 *  Copyright (C) 2009 Shigeru Kobayashi. All rights reserved.
 *  Copyright (C) 2013 Norbert Truchsess. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND LGPL-2.1-or-later
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_spi.hpp"

#include "firmata_extra.hpp"

#include <Boards.h>
#include <FirmataExpress.h>
#include <SPI.h>
#include <Scheduler.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t SPI_STACK_SIZE = 96; // Default is 128

// SPI constants
constexpr uint8_t SPI_BEGIN = 0x00; // Initialize the SPI bus for the given channel
constexpr uint8_t SPI_DEVICE_CONFIG = 0x01;
constexpr uint8_t SPI_TRANSFER = 0x02;
constexpr uint8_t SPI_WRITE = 0x03;
constexpr uint8_t SPI_READ = 0x04;
constexpr uint8_t SPI_REPLY = 0x05;
constexpr uint8_t SPI_END = 0x06;

constexpr uint8_t MAX_SPI_BUF_SIZE = 32;

} // namespace OASIS

FirmataSPI::FirmataSPI()
{
  Reset();
}

void FirmataSPI::Setup(void (*loopFunc)())
{
  Scheduler.startLoop(loopFunc, SPI_STACK_SIZE);
}

void FirmataSPI::Reset()
{
  // Reset SPI
  if (m_isSpiEnabled)
    SPI.end();

  // Reset state
  m_isSpiEnabled = false;

  for (uint8_t i = 0; i < SPI_MAX_DEVICES; ++i)
  {
    m_config[i].deviceIdChannel = static_cast<uint8_t>(-1);
    m_config[i].csPin = static_cast<uint8_t>(-1);
    m_config[i].used = false;
  }
}

void FirmataSPI::Loop()
{
  // TODO
  delay(1000);
}

bool FirmataSPI::EnableSpiPins()
{
  // Check MISO pin
  if (Firmata.getPinMode(PIN_SPI_MISO) == PIN_MODE_IGNORE)
    return false;

  // Check MOSI pin
  if (Firmata.getPinMode(PIN_SPI_MOSI) == PIN_MODE_IGNORE)
    return false;

  // Check SCK pin
  if (Firmata.getPinMode(PIN_SPI_SCK) == PIN_MODE_IGNORE)
    return false;

  // Mark pins as SPI so they are ignored in non-SPI data requests
  Firmata.setPinMode(PIN_SPI_MISO, PIN_MODE_SPI);
  Firmata.setPinMode(PIN_SPI_MOSI, PIN_MODE_SPI);
  Firmata.setPinMode(PIN_SPI_SCK, PIN_MODE_SPI);

  pinMode(PIN_SPI_MISO, INPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_SCK, OUTPUT);

  m_isSpiEnabled = true;

  return true;
}

void FirmataSPI::DisableSpiPins()
{
  m_isSpiEnabled = false;
  SPI.end();
}

void FirmataSPI::HandleSpiRequest(uint8_t command, uint8_t argc, uint8_t* argv)
{
  switch (command)
  {
    case SPI_BEGIN:
    {
      HandleSpiBegin(argc, argv);
      break;
    }

    case SPI_DEVICE_CONFIG:
    {
      HandleSpiConfig(argc, argv);
      break;
    }

    case SPI_END:
    {
      DisableSpiPins();
      break;
    }

    case SPI_READ:
    {
      HandleSpiTransfer(argc, argv, true, true);
      break;
    }

    case SPI_WRITE:
    {
      HandleSpiTransfer(argc, argv, false, false);
      break;
    }

    case SPI_TRANSFER:
    {
      HandleSpiTransfer(argc, argv, false, true);
      break;
    }

    default:
    {
      // TODO: Also send command
      Firmata.sendString("Unknown SPI command");
      break;
    }
  }
}

bool FirmataSPI::HandleSpiBegin(uint8_t argc, uint8_t* argv)
{
  if (!m_isSpiEnabled)
  {
    // Only channel 0 supported
    if (argc != 1 || *argv != 0)
    {
      Firmata.sendString("SPI_BEGIN: Only channel 0 supported");
      return false;
    }

    EnableSpiPins();

    SPI.begin();
  }

  return m_isSpiEnabled;
}

bool FirmataSPI::HandleSpiConfig(uint8_t argc, uint8_t* argv)
{
  if (argc < 10)
  {
    Firmata.sendString("Not enough data in SPI_DEVICE_CONFIG message");
    return false;
  }

  uint8_t index = static_cast<uint8_t>(-1); // the index where the new device will be added
  for (uint8_t i = 0; i < SPI_MAX_DEVICES; ++i)
  {
    if (m_config[i].deviceIdChannel == argv[0])
    {
      // This device exists already
      index = i;
    }
  }

  if (index == static_cast<uint8_t>(-1))
  {
    for (uint8_t i = 0; i < SPI_MAX_DEVICES; ++i)
    {
      if (m_config[i].used == false)
      {
        index = i;
      }
    }
  }

  if (index == static_cast<uint8_t>(-1))
  {
    Firmata.sendString("SPI_DEVICE_CONFIG: Max number of devices exceeded");
    return false;
  }

  // Check word size. Must be 0 (default) or 8
  if (argv[7] != 0 && argv[7] != 8)
  {
    Firmata.sendString("SPI_DEVICE_CONFIG: Only 8 bit words supported");
    return false;
  }

  uint8_t deviceIdChannel = argv[0];
  if ((deviceIdChannel & 0x3) != 0)
  {
    Firmata.sendString("SPI_DEVICE_CONFIG: Only channel 0 supported: ");
    return false;
  }

  if (argv[1] != 1)
  {
    Firmata.sendString("Only BitOrder = 1 and dataMode = 0 supported");
    return false;
  }

  m_config[index].deviceIdChannel = deviceIdChannel;
  m_config[index].dataModeBitOrder = argv[1];
  // Max speed ignored for now
  m_config[index].csPinOptions = argv[8];
  m_config[index].csPin = argv[9];
  m_config[index].used = true;

  return true;
}

void FirmataSPI::HandleSpiTransfer(uint8_t argc, uint8_t* argv, bool dummySend, bool sendReply)
{
  if (!m_isSpiEnabled)
  {
    Firmata.sendString("SPI not enabled.");
    return;
  }

  uint8_t data[MAX_SPI_BUF_SIZE];

  // Make sure we have enough data. No data bytes is only allowed in read-only mode
  if (dummySend ? argc < 4 : argc < 6)
  {
    Firmata.sendString("Not enough data in SPI message");
    return;
  }

  const int index = GetConfigIndexForDevice(argv[0]);
  if (index < 0)
  {
    // TODO: Also send argv[0]
    Firmata.sendString("SPI_TRANSFER: Unknown deviceId specified");
    return;
  }

  int j = 0;

  // In read-only mode set buffer to 0, otherwise fill buffer from request
  if (dummySend)
  {
    for (uint8_t i = 0; i < argv[3]; i += 1)
    {
      data[j++] = 0;
    }
  }
  else
  {
    for (uint8_t i = 4; i < argc; i += 2)
    {
      data[j++] = argv[i] + (argv[i + 1] << 7);
    }
  }

  digitalWrite(m_config[index].csPin, LOW);

  SPI.transfer(data, j);

  if (argv[2] != 0)
  {
    // Default is deselect, so only skip this if the value is 0
    digitalWrite(m_config[index].csPin, HIGH);
  }

  if (sendReply)
  {
    Firmata.startSysex();
    Firmata.write(SPI_DATA);
    Firmata.write(SPI_REPLY);
    Firmata.write(argv[0]);
    Firmata.write(argv[1]);
    Firmata.write(j);

    for (uint8_t i = 0; i < j; ++i)
      Firmata.sendValueAsTwo7bitBytes(data[i]);

    Firmata.endSysex();
  }
}

int FirmataSPI::GetConfigIndexForDevice(uint8_t deviceIdChannel)
{
  for (uint8_t i = 0; i < SPI_MAX_DEVICES; i++)
  {
    if (m_config[i].deviceIdChannel == deviceIdChannel)
    {
      return static_cast<int>(i);
    }
  }

  return -1;
}

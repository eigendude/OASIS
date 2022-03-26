/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the FirmataExpress project and the AGPL-3. License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_i2c.hpp"

#include "firmata_callbacks.hpp"

#include <stddef.h>

#include <FirmataExpress.h>
#include <Scheduler.h>
#include <Wire.h>

using namespace OASIS;

namespace OASIS
{

// Threading constants
constexpr size_t I2C_STACK_SIZE = 64; // Default is 128

} // namespace OASIS

void FirmataI2C::Setup(void (*loopFunc)())
{
  Scheduler.startLoop(loopFunc, I2C_STACK_SIZE);
}

void FirmataI2C::Reset()
{
  if (m_isI2CEnabled)
    DisableI2CPins();
}

void FirmataI2C::Loop()
{
  // Report I2C data for all device with read continuous mode enabled
  if (m_queryIndex > -1)
  {
    for (uint8_t i = 0; i < m_queryIndex + 1; ++i)
      ReadAndReportI2CData(m_query[i].addr, m_query[i].reg, m_query[i].bytes, m_query[i].stopTX);
  }
}

void FirmataI2C::EnableI2CPins()
{
  // Is there a faster way to do this? Would probably require importing
  // Arduino.h to get SCL and SDA pins
  for (uint8_t i = 0; i < TOTAL_PINS; ++i)
  {
    if (IS_PIN_I2C(i))
    {
      // Mark pins as i2c so they are ignore in non I2C data requests
      FirmataCallbacks::SetPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  m_isI2CEnabled = true;

  Wire.begin();
}

void FirmataI2C::DisableI2CPins()
{
  m_isI2CEnabled = false;

  // Disable read continuous mode for all devices
  m_queryIndex = -1;
}

void FirmataI2C::ReadAndReportI2CData(uint8_t address,
                                      int theRegister,
                                      uint8_t numBytes,
                                      uint8_t stopTX)
{
  // Allow I2C requests that don't require a register read. For example, some
  // devices using an interrupt pin to signify new data available do not
  // always require the register read so upon interrupt you call
  // Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED)
  {
    Wire.beginTransmission(address);
    Wire.write(static_cast<uint8_t>(theRegister));

    Wire.endTransmission(stopTX); // Default = true

    // Do not set a value of 0
    if (m_i2cReadDelayTime > 0)
    {
      // Delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(m_i2cReadDelayTime);
    }
  }
  else
  {
    // Fill the register with a dummy value
    theRegister = 0;
  }

  // All bytes are returned in requestFrom
  Wire.requestFrom(address, numBytes);

  // Check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available())
  {
    Firmata.sendString("I2C: Too many bytes received");
  }
  else if (numBytes > Wire.available())
  {
    Firmata.sendString("I2C: Too few bytes received");
  }

  m_i2cRxData[0] = address;
  m_i2cRxData[1] = theRegister;

  for (unsigned int i = 0; i < numBytes && Wire.available(); ++i)
    m_i2cRxData[2 + i] = Wire.read();

  // Send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, m_i2cRxData);
}

unsigned int FirmataI2C::GetI2CQueryCount() const
{
  // m_queryIndex starts at -1, and it can't go more negative
  return static_cast<unsigned int>(m_queryIndex + 1);
}

uint8_t FirmataI2C::GetI2CQueryAddress(uint8_t queryIndex) const
{
  return m_query[queryIndex].addr;
}

void FirmataI2C::AddI2CQuery(uint8_t slaveAddress, int slaveRegister, uint8_t data, uint8_t stopTX)
{
  unsigned int newQueryIndex = static_cast<unsigned int>(++m_queryIndex);

  m_query[newQueryIndex].addr = slaveAddress;
  m_query[newQueryIndex].reg = slaveRegister;
  m_query[newQueryIndex].bytes = data;
  m_query[newQueryIndex].stopTX = stopTX;
}

void FirmataI2C::RemoveI2CQuery(uint8_t queryIndex)
{
  m_query[queryIndex].addr = m_query[queryIndex + 1].addr;
  m_query[queryIndex].reg = m_query[queryIndex + 1].reg;
  m_query[queryIndex].bytes = m_query[queryIndex + 1].bytes;
  m_query[queryIndex].stopTX = m_query[queryIndex + 1].stopTX;
}

void FirmataI2C::SetPreviousI2CQuery()
{
  --m_queryIndex;
}

void FirmataI2C::DisableI2CReporting()
{
  m_queryIndex = -1;
}

void FirmataI2C::SetI2CReadDelayTime(unsigned int delayTime)
{
  m_i2cReadDelayTime = delayTime;
}

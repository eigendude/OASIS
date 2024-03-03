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

#include "telemetrix_i2c.hpp"

#include "drivers/i2c_port.hpp"
#include "telemetrix_reports.hpp"

#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixI2C::I2CBegin(uint8_t i2cPortIndex)
{
  if (i2cPortIndex < MAX_I2C_PORTS && m_ports[i2cPortIndex] == nullptr)
  {
    I2CPort* newPort = I2CPort::CreateI2CPort(i2cPortIndex);
    if (newPort != nullptr)
      m_ports[i2cPortIndex] = newPort;
  }
}

void TelemetrixI2C::I2CRead(uint8_t i2cPortIndex,
                            uint8_t i2cAddress,
                            uint8_t theRegister,
                            uint8_t byteCount,
                            bool stopTransmitting,
                            bool writeByte)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  // Data in the incoming message:
  // address, [0]
  // register, [1]
  // number of bytes, [2]
  // stop transmitting flag [3]
  // i2c port [4]
  // write the register [5]

  uint8_t i2cReportMessage[64];

  const int i2cDataRead = i2cPort.I2CRead(i2cAddress, theRegister, byteCount, stopTransmitting,
                                          writeByte, i2cReportMessage + 6);

  // Check to be sure correct number of bytes were returned by slave
  if (byteCount < i2cDataRead)
  {
    const uint8_t errorReportMessage[4] = {3, I2C_TOO_FEW_BYTES_RCVD, 1, i2cAddress};
    Serial.write(errorReportMessage, 4);
    return;
  }
  else if (byteCount > i2cDataRead)
  {
    const uint8_t errorReportMessage[4] = {3, I2C_TOO_MANY_BYTES_RCVD, 1, i2cAddress};
    Serial.write(errorReportMessage, 4);
    return;
  }

  // Packet length
  i2cReportMessage[0] = byteCount + 5;

  // Report type
  i2cReportMessage[1] = I2C_READ_REPORT;

  // I2C_port
  i2cReportMessage[2] = i2cPort.I2CPortIndex();

  // Number of bytes read
  i2cReportMessage[3] = byteCount; // number of bytes

  // Device address
  i2cReportMessage[4] = i2cAddress;

  // Device register
  i2cReportMessage[5] = theRegister;

  // Send slave address, register and received bytes
  for (unsigned int i = 0; i < i2cDataRead + 6; ++i)
    Serial.write(i2cReportMessage[i]);
}

void TelemetrixI2C::I2CWrite(uint8_t i2cPortIndex,
                             uint8_t i2cAddress,
                             uint8_t byteCount,
                             const uint8_t* data)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  i2cPort.I2CWrite(i2cAddress, byteCount, data);
}

void TelemetrixI2C::BeginCCS811(uint8_t i2cPortIndex, uint8_t i2cAddress)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  i2cPort.BeginCCS811(i2cAddress);
}

void TelemetrixI2C::EndCCS811(uint8_t i2cPortIndex, uint8_t i2cAddress)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  i2cPort.EndCCS811(i2cAddress);
}

void TelemetrixI2C::BeginMPU6050(uint8_t i2cPortIndex, uint8_t i2cAddress)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  i2cPort.BeginMPU6050(i2cAddress);
}

void TelemetrixI2C::EndMPU6050(uint8_t i2cPortIndex, uint8_t i2cAddress)
{
  if (m_ports[i2cPortIndex] == nullptr)
    return;

  I2CPort& i2cPort = *m_ports[i2cPortIndex];

  i2cPort.EndMPU6050(i2cAddress);
}

void TelemetrixI2C::ScanSensors()
{
  for (unsigned int i = 0; i < MAX_I2C_PORTS; ++i)
  {
    if (m_ports[i] != nullptr)
    {
      m_ports[i]->ScanCCS811Sensors(
          [](uint8_t i2cPortIndex, uint8_t i2cAddress, uint16_t co2Ppb, uint16_t tvocPpb)
          {
            //
            // Report message
            //
            // byte 0 = packet length
            // byte 1 = report type
            // byte 2 = I2C port
            // byte 3 = I2C address
            // byte 4 = C02 ppb MSB
            // byte 5 = C02 ppb LSB
            // byte 6 = TVOC ppb MSB
            // byte 7 = TVOC ppb LSB
            //
            const uint8_t reportMessage[] = {
                7,           AQ_CO2_TVOC_REPORT, i2cPortIndex, i2cAddress,
                co2Ppb >> 8, co2Ppb & 0xff,      tvocPpb >> 8, tvocPpb & 0xff,
            };

            Serial.write(reportMessage, 8);
          });

      m_ports[i]->ScanMPU6050Sensors(
          [](uint8_t i2cPortIndex, uint8_t i2cAddress, int16_t ax, int16_t ay, int16_t az,
             int16_t gx, int16_t gy, int16_t gz)
          {
            //
            // Report message
            //
            // byte 0 = packet length
            // byte 1 = report type
            // byte 2 = I2C port
            // byte 3 = I2C address
            // byte 4 = ax MSB
            // byte 5 = ax LSB
            // byte 6 = ay MSB
            // byte 7 = ay LSB
            // byte 8 = az MSB
            // byte 9 = az LSB
            // byte 10 = gx MSB
            // byte 11 = gx LSB
            // byte 12 = gy MSB
            // byte 13 = gy LSB
            // byte 14 = gz MSB
            // byte 15 = gz LSB
            //
            const uint8_t reportMessage[] = {
                15,
                IMU_6_AXIS_REPORT,
                i2cPortIndex,
                i2cAddress,
                static_cast<uint16_t>(ax) >> 8,
                static_cast<uint16_t>(ax) & 0xff,
                static_cast<uint16_t>(ay) >> 8,
                static_cast<uint16_t>(ay) & 0xff,
                static_cast<uint16_t>(az) >> 8,
                static_cast<uint16_t>(az) & 0xff,
                static_cast<uint16_t>(gx) >> 8,
                static_cast<uint16_t>(gx) & 0xff,
                static_cast<uint16_t>(gy) >> 8,
                static_cast<uint16_t>(gy) & 0xff,
                static_cast<uint16_t>(gz) >> 8,
                static_cast<uint16_t>(gz) & 0xff,
            };

            Serial.write(reportMessage, 16);
          });
    }
  }
}

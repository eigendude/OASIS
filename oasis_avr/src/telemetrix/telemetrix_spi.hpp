/*
 *  Copyright (C) 2022-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS
{
class TelemetrixSPI
{
public:
  void InitSPI(uint8_t pinCount, const uint8_t* pins);
  void WriteBlockingSPI(uint8_t byteCount, const uint8_t* data);
  void ReadBlockingSPI(uint8_t byteCount, uint8_t readRegister);
  void SetFormatSPI(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
  void SPICSControl(uint8_t csPin, uint8_t csState);

private:
  // A buffer to hold spi report data
  uint8_t* m_spiReportMessage{nullptr};
};
} // namespace OASIS

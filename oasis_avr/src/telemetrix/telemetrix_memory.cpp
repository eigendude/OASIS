/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_memory.hpp"

#include "telemetrix_reports.hpp"
#include "utils/memory_utils.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>

using namespace OASIS;

void TelemetrixMemory::ScanMemory()
{
  if (m_reportIntervalMs == 0)
  {
    // Memory reporting is disabled
    return;
  }

  if (!m_reportTimer.IsExpired())
  {
    // Timer hasn't elapsed yet
    return;
  }

  m_reportTimer.SetTimeout(m_reportIntervalMs);

  const size_t totalRam = MemoryUtils::GetTotalRAM();
  const size_t staticDataSize = MemoryUtils::GetStaticDataSize();
  const size_t heapSize = MemoryUtils::GetHeapSize();
  const size_t stackSize = MemoryUtils::GetStackSize();
  const size_t freeRam = MemoryUtils::GetFreeRAM();
  const size_t freeHeap = MemoryUtils::GetFreeHeap();

  //
  // Report message
  //
  // byte 0 = packet length
  // byte 1 = report type
  //
  // byte 2 = high order byte of total RAM
  // byte 3 = middle byte of total RAM
  // byte 4 = low order byte of total RAM
  //
  // byte 5 = high order byte of static data size
  // byte 6 = middle byte of static data size
  // byte 7 = low order byte of static data size
  //
  // byte 8 = high order byte of heap size
  // byte 9 = middle byte of heap size
  // byte 10 = low order byte of heap size
  //
  // byte 11 = high order byte of stack size
  // byte 12 = middle byte of stack size
  // byte 13 = low order byte of stack size
  //
  // byte 14 = high order byte of free RAM
  // byte 15 = middle byte of free RAM
  // byte 16 = low order byte of free RAM
  //
  // byte 17 = high order byte of free heap
  // byte 18 = middle byte of free heap
  // byte 19 = low order byte of free heap
  //
  const uint8_t reportMessage[20] = {
      19,
      MEMORY_REPORT,

      static_cast<uint8_t>((totalRam >> 16) & 0xFF),
      static_cast<uint8_t>((totalRam >> 8) & 0xFF),
      static_cast<uint8_t>(totalRam & 0xFF),

      static_cast<uint8_t>((staticDataSize >> 16) & 0xFF),
      static_cast<uint8_t>((staticDataSize >> 8) & 0xFF),
      static_cast<uint8_t>(staticDataSize & 0xFF),

      static_cast<uint8_t>((heapSize >> 16) & 0xFF),
      static_cast<uint8_t>((heapSize >> 8) & 0xFF),
      static_cast<uint8_t>(heapSize & 0xFF),

      static_cast<uint8_t>((stackSize >> 16) & 0xFF),
      static_cast<uint8_t>((stackSize >> 8) & 0xFF),
      static_cast<uint8_t>(stackSize & 0xFF),

      static_cast<uint8_t>((freeRam >> 16) & 0xFF),
      static_cast<uint8_t>((freeRam >> 8) & 0xFF),
      static_cast<uint8_t>(freeRam & 0xFF),

      static_cast<uint8_t>((freeHeap >> 16) & 0xFF),
      static_cast<uint8_t>((freeHeap >> 8) & 0xFF),
      static_cast<uint8_t>(freeHeap & 0xFF),
  };

  Serial.write(reportMessage, 20);
}

void TelemetrixMemory::ResetData()
{
  m_reportTimer.Reset();
  m_reportIntervalMs = 0;
}

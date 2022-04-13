/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_diagnostics.hpp"

#include "firmata_extra.hpp"
#include "utils/memory_utils.hpp"

#include <FirmataExpress.h>

using namespace OASIS;

void FirmataDiagnostics::Sample()
{
  if (m_reportPeriodMs > 0)
  {
    if (m_reportTimer.IsExpired())
    {
      m_reportTimer.SetTimeout(m_reportPeriodMs);

      const size_t totalRam = MemoryUtils::GetTotalRAM();
      const size_t staticDataSize = MemoryUtils::GetStaticDataSize();
      const size_t heapSize = MemoryUtils::GetHeapSize();
      const size_t stackSize = MemoryUtils::GetStackSize();
      const size_t freeRam = MemoryUtils::GetFreeRAM();
      const size_t freeHeap = MemoryUtils::GetFreeHeap();

      Firmata.write(START_SYSEX);
      Firmata.write(FIRMATA_MEMORY_DATA);

      Firmata.write(static_cast<uint8_t>(totalRam & 0x7F));
      Firmata.write(static_cast<uint8_t>((totalRam >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((totalRam >> 14) & 0x7F));

      Firmata.write(static_cast<uint8_t>(staticDataSize & 0x7F));
      Firmata.write(static_cast<uint8_t>((staticDataSize >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((staticDataSize >> 14) & 0x7F));

      Firmata.write(static_cast<uint8_t>(heapSize & 0x7F));
      Firmata.write(static_cast<uint8_t>((heapSize >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((heapSize >> 14) & 0x7F));

      Firmata.write(static_cast<uint8_t>(stackSize & 0x7F));
      Firmata.write(static_cast<uint8_t>((stackSize >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((stackSize >> 14) & 0x7F));

      Firmata.write(static_cast<uint8_t>(freeRam & 0x7F));
      Firmata.write(static_cast<uint8_t>((freeRam >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((freeRam >> 14) & 0x7F));

      Firmata.write(static_cast<uint8_t>(freeHeap & 0x7F));
      Firmata.write(static_cast<uint8_t>((freeHeap >> 7) & 0x7F));
      Firmata.write(static_cast<uint8_t>((freeHeap >> 14) & 0x7F));

      Firmata.write(END_SYSEX);
    }
  }
}

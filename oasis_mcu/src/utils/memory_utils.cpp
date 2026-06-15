/*
 *  Copyright (C) 2021-2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "memory_utils.hpp"

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#include "avr_defines.hpp"

#include <avr/common.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <Arduino.h>
#endif

#if !defined(ARDUINO_BOARD_RAM_SIZE)
#error "ARDUINO_BOARD_RAM_SIZE should have been defined by the build system"
#endif

using namespace OASIS;

size_t MemoryUtils::GetTotalRAM()
{
#if defined(ARDUINO_ARCH_ESP32)
  return ESP.getHeapSize();
#else
  return ARDUINO_BOARD_RAM_SIZE;
#endif
}

size_t MemoryUtils::GetStaticDataSize()
{
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
  return reinterpret_cast<size_t>(__heap_start);
#else
  return 0;
#endif
}

size_t MemoryUtils::GetHeapSize()
{
#if defined(ARDUINO_ARCH_ESP32)
  const size_t totalRam = GetTotalRAM();
  const size_t freeRam = GetFreeRAM();

  return (totalRam > freeRam ? totalRam - freeRam : 0);
#else
  const uintptr_t heapPointer = GetHeapPointer();

  const size_t heapSize =
      (heapPointer > GetStaticDataSize() ? static_cast<size_t>(heapPointer - GetStaticDataSize())
                                         : 0);

  return heapSize;
#endif
}

size_t MemoryUtils::GetStackSize()
{
#if defined(ARDUINO_ARCH_ESP32)
  return 0;
#else
  const uintptr_t stackPointer = GetStackPointer();

  const size_t stackSize =
      (GetTotalRAM() > stackPointer ? static_cast<size_t>(GetTotalRAM() - stackPointer) : 0);

  return stackSize;
#endif
}

size_t MemoryUtils::GetFreeRAM()
{
#if defined(ARDUINO_ARCH_ESP32)
  return ESP.getFreeHeap();
#else
  const uintptr_t heapPointer = GetHeapPointer();
  const uintptr_t stackPointer = GetStackPointer();

  const size_t freeRam =
      (stackPointer > heapPointer ? static_cast<size_t>(stackPointer - heapPointer) : 0);

  return freeRam;
#endif
}

size_t MemoryUtils::GetFreeHeap()
{
#if defined(ARDUINO_ARCH_ESP32)
  return ESP.getMaxAllocHeap();
#else
  size_t total = 0;

  // Skip check if memory has not been allocated yet
  if (__brkval != nullptr)
  {
    for (__freelist* current = __flp; current != nullptr; current = current->nx)
    {
      // Add two bytes for the memory block's header
      total += 2;
      total += current->sz;
    }
  }

  return total;
#endif
}

uintptr_t MemoryUtils::GetHeapPointer()
{
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
  uintptr_t heapPointer = 0;

  // If memory hasn't been allocated, get the heap pointer from __heap_start.
  // Otherwise, use the heap allocation pointer, __brkval.
  if (__brkval == nullptr)
    heapPointer = reinterpret_cast<uintptr_t>(&__heap_start);
  else
    heapPointer = reinterpret_cast<uintptr_t>(__brkval);

  return heapPointer;
#else
  return 0;
#endif
}

uintptr_t MemoryUtils::GetStackPointer()
{
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
  const uintptr_t stackPointer = static_cast<uintptr_t>(SP);

  return stackPointer;
#else
  return 0;
#endif
}

/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "memory_utils.hpp"

#include "avr_defines.hpp"

#include <avr/common.h>

#if !defined(ARDUINO_BOARD_RAM_SIZE)
#error "ARDUINO_BOARD_RAM_SIZE should have been defined by the build system"
#endif

using namespace OASIS;

size_t MemoryUtils::GetTotalRAM()
{
  return ARDUINO_BOARD_RAM_SIZE;
}

size_t MemoryUtils::GetStaticDataSize()
{
  return reinterpret_cast<size_t>(__heap_start);
}

size_t MemoryUtils::GetHeapSize()
{
  const uintptr_t heapPointer = GetHeapPointer();

  const size_t heapSize =
      (heapPointer > GetStaticDataSize() ? static_cast<size_t>(heapPointer - GetStaticDataSize())
                                         : 0);

  return heapSize;
}

size_t MemoryUtils::GetStackSize()
{
  const uintptr_t stackPointer = GetStackPointer();

  const size_t stackSize =
      (GetTotalRAM() > stackPointer ? static_cast<size_t>(GetTotalRAM() - stackPointer) : 0);

  return stackSize;
}

size_t MemoryUtils::GetFreeRAM()
{
  const uintptr_t heapPointer = GetHeapPointer();
  const uintptr_t stackPointer = GetStackPointer();

  const size_t freeRam =
      (stackPointer > heapPointer ? static_cast<size_t>(stackPointer - heapPointer) : 0);

  return freeRam;
}

size_t MemoryUtils::GetFreeHeap()
{
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
}

uintptr_t MemoryUtils::GetHeapPointer()
{
  uintptr_t heapPointer = 0;

  // If memory hasn't been allocated, get the heap pointer from __heap_start.
  // Otherwise, use the heap allocation pointer, __brkval.
  if (__brkval == nullptr)
    heapPointer = reinterpret_cast<uintptr_t>(&__heap_start);
  else
    heapPointer = reinterpret_cast<uintptr_t>(__brkval);

  return heapPointer;
}

uintptr_t MemoryUtils::GetStackPointer()
{
  const uintptr_t stackPointer = static_cast<uintptr_t>(SP);

  return stackPointer;
}

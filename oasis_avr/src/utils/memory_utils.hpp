/*
 *  Copyright (C) 2021-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace OASIS
{

class MemoryUtils
{
public:
  /*!
   * \brief Get the total RAM size, in bytes
   */
  static size_t GetTotalRAM();

  /*!
   * \brief Get the size of the static data section, before the heap begins
   */
  static size_t GetStaticDataSize();

  /*!
   * \brief Get the size of the heap, starting from the static data section
   * and growing up
   */
  static size_t GetHeapSize();

  /*!
   * \brief Get the size of the stack, starting from the end of RAM and growing
   * down
   */
  static size_t GetStackSize();

  /*!
   * \brief Get the amount of free RAM between the heap (growing up) and the
   * stack (growing down)
   */
  static size_t GetFreeRAM();

  /*!
   * \brief Get the amount of RAM available from de-allocated fragments of
   * memory
   *
   * When RAM is de-allocted, the memory fragment is added to a list maintained
   * by avr-libc's memory allocation routines.
   */
  static size_t GetFreeHeap();

private:
  /*!
   * \brief Get a pointer to the head of the heap in RAM
   */
  static uintptr_t GetHeapPointer();

  /*!
   * \brief Get a pointer to the head of the stack in RAM
   */
  static uintptr_t GetStackPointer();
};

} // namespace OASIS

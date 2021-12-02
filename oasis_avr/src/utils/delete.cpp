/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "delete.hpp"

#if defined(__AVR__) && __cpp_sized_deallocation

void operator delete(void* ptr, size_t) noexcept
{
  delete ptr;
}

void operator delete[](void* ptr, size_t) noexcept
{
  delete[] ptr;
}

#endif

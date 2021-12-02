/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

//
// This declaration fixes the error:
//
//   undefined reference to `operator delete(void*, unsigned int)'
//
// For more info, see:
//
//   https://forum.arduino.cc/t/undefined-reference-to-operator-delete-void-unsigned-int/620428
//

#if defined(__AVR__) && __cpp_sized_deallocation

#include <new>
#include <stddef.h>

void operator delete(void* ptr, size_t) noexcept;
void operator delete[](void* ptr, size_t) noexcept;

#endif

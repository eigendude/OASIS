/*
 *  Copyright (C) 2021-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stddef.h>

#include <avr/io.h>

//
// AVR memory types
//

// The free list structure as maintained by the avr-libc memory
// routines
struct __freelist
{
  size_t sz;
  __freelist* nx;
};

//
// AVR memory variables
//

// Current end of heap (0 if no allocation yet)
extern void* __brkval;

// Beginning of allocation heap, used in case __brkval is 0
extern unsigned int __heap_start;

// The head of the free list structure for free memory in the heap
extern __freelist* __flp;

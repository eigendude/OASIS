################################################################################
#
#  Copyright (C) 2021-2023 Garrett Brown
#  This file is part of OASIS - https://github.com/eigendude/OASIS
#
#  SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
#  See DOCS/LICENSING.md for more information.
#
################################################################################

#
# Memory statistics for a microcontroller CPU
#
# The memory model for AVR-like microcontrollers is relatively simple. The
# beginning of RAM is allocated for static variables. The heap starts afterward,
# and grows up toward the end of RAM. The stack starts at the end of RAM, and
# growns down toward the heap.
#

# The message header
std_msgs/Header header

# Total RAM size, in bytes
uint32 total_ram

# Size of the static data section, before the heap begins
uint32 static_data_size

# Size of the heap, starting from the static data section and growing up
uint32 heap_size

# Size of the stack, starting from the end of RAM and growing down
uint32 stack_size

# Amount of free RAM between the heap (growing up) and the stack (growing down)
uint32 free_ram

# Amount of RAM available from de-allocated fragments of memory.
#
# When RAM is de-allocted, the memory fragment is added to a list maintained
# by avr-libc's memory allocation routines.
#
# This parameter is the total size of free memory fragments in the free memory
# list.
uint32 free_heap

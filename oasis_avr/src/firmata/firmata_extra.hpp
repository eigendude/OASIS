/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

namespace OASIS
{

// Extended command set using sysex (0-127/0x00-0x7F)
constexpr int FIRMATA_MEMORY_CONFIG = 0x64; // Enable/disable reporting of memory statistics
constexpr int FIRMATA_MEMORY_DATA = 0x65; // Receive a memory statistics report

} // namespace OASIS

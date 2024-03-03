/*
 *  Copyright (C) 2021-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from ConfugurableFirmata under the LGPL 2.1 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2010-2011 Paul Stoffregen. All rights reserved.
 *  Copyright (C) 2009 Shigeru Kobayashi. All rights reserved.
 *  Copyright (C) 2013 Norbert Truchsess. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND LGPL-2.1-or-later
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

//
// This file is for functionality found in Firmata forks but not present in
// FirmataExpress, as well as new functionality added specifically in this
// project.
//

#include <stdint.h>

namespace OASIS
{

//
// Diagnostics functionality
//

// Extended command set using sysex (0-127/0x00-0x7F)
constexpr int FIRMATA_MEMORY_CONFIG = 0x66; // Enable/disable reporting of memory statistics
constexpr int FIRMATA_MEMORY_DATA = 0x67; // Receive a memory statistics report

//
// Imported from ConfigurableFirmata
//

// Pin configured for SPI (this value was 0x0C in ConfigurableFirmata, but 0x0C
// is taken by PIN_MODE_SONAR in FirmataExpress)
constexpr uint8_t PIN_MODE_SPI = 0x10;

// SPI Commands start with this byte
constexpr uint8_t SPI_DATA = 0x68;

//
// CPU fan support
//

// Pin configured to drive a CPU fan with a PWM signal. Control is done via
// the standard analog write message.
constexpr uint8_t PIN_MODE_CPU_FAN_PWM = 0x11;

// Pin configured to sense the speed of a CPU fan using the tachometer signal
constexpr uint8_t PIN_MODE_CPU_FAN_TACH = 0x12;

// Sysex command to report CPU fan speeds
constexpr uint8_t FIRMATA_CPU_FAN_RPM = 0x53;

} // namespace OASIS

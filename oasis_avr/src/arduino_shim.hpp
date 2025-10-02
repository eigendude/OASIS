/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

//
// Some versions of the megaAVR core declare friends in IPAddress.h using
// unqualified names like "friend ::EthernetClass;". When those headers are used
// without the associated libraries that normally provide the declarations, the
// compiler errors out before we can include Arduino.h. Forward declaring the
// missing types restores compatibility with these cores.
//

class EthernetClass;
class DhcpClass;
class DNSClient;

#include <Arduino.h>

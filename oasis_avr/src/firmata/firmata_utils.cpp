/*
 *  Copyright (C) 2021-2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from FirmataExpress under the AGPL 3.0 License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "firmata_utils.hpp"

#include <FirmataExpress.h>
#include <WString.h>

using namespace OASIS;

void FirmataUtils::PrintData(const char* id, long data)
{
  char myArray[64]{};

  String myString = String(data);
  myString.toCharArray(myArray, 64);

  Firmata.sendString(id);
  Firmata.sendString(myArray);
}

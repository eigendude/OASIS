/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "NetworkUtils.h"

#include <unistd.h>

using namespace OASIS;
using namespace UTILS;

std::string NetworkUtils::GetHostName()
{
  char hostName[128];
  if (gethostname(hostName, sizeof(hostName)) == 0)
    return hostName;

  return "";
}

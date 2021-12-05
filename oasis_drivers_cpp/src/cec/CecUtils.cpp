/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "CecUtils.h"

#include "utils/StringUtils.h"

#include <sstream>

#include <libcec/cectypes.h>

using namespace OASIS;
using namespace OASIS::CEC;

std::string CecUtils::ByteToHexString(uint8_t value)
{
  return UTILS::StringUtils::ToHexString(value);
}

std::string CecUtils::PhysicalAdressToHexString(uint16_t address)
{
  return UTILS::StringUtils::ToHexString(address);
}

std::string CecUtils::ParametersToHexArray(const ::CEC::cec_datapacket& parameters)
{
  std::ostringstream stringBuffer;
  stringBuffer << "[";

  for (int i = 0; i < static_cast<int>(parameters.size); ++i)
  {
    stringBuffer << ByteToHexString(parameters[i]);
    if (i < static_cast<int>(parameters.size) - 1)
      stringBuffer << ", ";
  }

  stringBuffer << "]";

  return stringBuffer.str();
}

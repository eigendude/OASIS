/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <string>

namespace OASIS
{
namespace CEC
{

class ICecAdapter
{
public:
  virtual ~ICecAdapter() = default;

  // Getters
  virtual const std::string& GetDevicePath() const = 0;
  virtual const std::string& GetDeviceNode() const = 0;
};

}
}

/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "servomotor.hpp"

using namespace OASIS;

uint8_t Servomotor::Attach(uint8_t pin, int min, int max)
{
  return m_servo.attach(pin, min, max);
}

void Servomotor::Detach()
{
  m_servo.detach();
}

void Servomotor::Write(int value)
{
  m_servo.write(value);
}

bool Servomotor::Attached()
{
  return m_servo.attached();
}

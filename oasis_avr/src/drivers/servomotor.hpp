/*
 *  Copyright (C) 2022-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <stdint.h>

#include <Servo.h>

namespace OASIS
{

class Servomotor
{
public:
  /*!
   * \brief Attach the given pin to the next free channel, sets pinMode, returns
   * channel number or 0 if failure
   */
  uint8_t Attach(uint8_t pin, int min, int max);

  void Detach();

  /*!
   * \brief If value is < 200 its treated as an angle, otherwise as pulse width
   * in microseconds
   */
  void Write(int value);

  /*!
   * \brief Returns true if this servo is attached, otherwise false
   */
  bool Attached();

private:
  Servo m_servo;
};

} // namespace OASIS

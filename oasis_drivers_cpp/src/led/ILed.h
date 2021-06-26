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
namespace LED
{

/*!
 * \brief LED abstraction
 */
class ILed
{
public:
  virtual ~ILed() = default;

  /*!
   * \brief Open the LED's resource
   *
   * \return true if the LED is ready to be controlled, false otherwise
   */
  virtual bool Open() = 0;

  /*!
   * \brief Close the LED's resource
   */
  virtual void Close() = 0;

  /*!
   * \brief Update the LED's brightness
   *
   * \param force TEMPORARY parameter used to force an update, even if the
   *               brightness hasn't changed
   *
   * TODO: Replace 'force' parameter with a less hacky workaround.
   */
  virtual void Update(bool force) = 0; // TODO: Remove force parameter

  /*!
   * \brief Set the LED to full brightness
   */
  void Enable() { m_brightness = 1.0f; }

  /*!
   * \brief Set the LED to off
   */
  void Disable() { m_brightness = 0.0f; }

  /*!
   * \brief Set the LED brightness to the specified ratio
   *
   * TODO: Fractional brightness ratios are not yet supported.
   *
   * \param brightnessRatio The specified brightness ratio
   */
  void SetBrightness(float brightnessRatio) { m_brightness = brightnessRatio; }

protected:
  // State parameters
  float m_brightness = 0.0f;
};

}
}

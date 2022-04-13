/*
 *  Copyright (C) 2021 Garrett Brown
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
#pragma once

#include "firmata_subsystem.hpp"
#include "utils/timer.hpp"

#include <stdint.h>

#include <Boards.h>

namespace OASIS
{

class FirmataDigital : public FirmataSubsystem
{
public:
  // Implementation of FirmataSubsystem
  void Loop() override;

  // Digital pin functions
  void SetDigitalPinMode(uint8_t digitalPin, int pinMode);
  void DisableDigitalReporting(uint8_t digitalPin);
  void EnableDigitalInput(uint8_t digitalPort, bool enable);
  void SendPort(uint8_t digitalPort);
  void DigitalWrite(uint8_t digitalPort, int portValue);
  void PWMWrite(uint8_t digitalPin, int analogValue);

private:
  // Digital pin functions
  void OutputPort(uint8_t digitalPort, uint8_t portValue, bool forceSend);
  /*!
   * \brief Check all the active digital inputs for change of state
   */
  void CheckDigitalInputs(bool forceSend);

  // Digital input ports
  uint8_t m_reportPINs[TOTAL_PORTS]{}; // 1 = report this port, 0 = silence
  uint8_t m_previousPINs[TOTAL_PORTS]{}; // Previous 8 bits sent

  // Pin configuration
  uint8_t m_portConfigInputs[TOTAL_PORTS]{}; // Each bit: 1 = pin in INPUT, 0 = anything else

  // Timing parameters
  Timer m_reportTimer;
};

} // namespace OASIS

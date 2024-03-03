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

#include "firmata_subsystem.hpp"

#include <stdint.h>

namespace OASIS
{

class FirmataSPI : public FirmataSubsystem
{
public:
  // SPI functions
  bool EnableSpiPins();
  /*!
   * \brief Disable the SPI pins so they can be used for other functions
   */
  void DisableSpiPins();
  bool IsSpiEnabled() const { return m_isSpiEnabled; }
  void HandleSpiRequest(uint8_t command, uint8_t argc, uint8_t* argv);

private:
  // SPI types
  /*!
   * \brief SPI data
   */
  struct spi_device_config
  {
    uint8_t deviceIdChannel;
    uint8_t dataModeBitOrder;
    uint8_t csPinOptions;
    uint8_t csPin;
    bool used;
  };

  // SPI functions
  bool HandleSpiBegin(uint8_t argc, uint8_t* argv);
  bool HandleSpiConfig(uint8_t argc, uint8_t* argv);
  void HandleSpiTransfer(uint8_t argc, uint8_t* argv, bool dummySend, bool sendReply);
  int GetConfigIndexForDevice(uint8_t deviceIdChannel);

  // SPI constants
  static constexpr uint8_t SPI_MAX_DEVICES = 8;

  // SPI state
  spi_device_config m_config[SPI_MAX_DEVICES]{};
  bool m_isSpiEnabled = false;
};

} // namespace OASIS

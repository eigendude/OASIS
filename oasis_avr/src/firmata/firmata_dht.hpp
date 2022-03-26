/*
 *  Copyright (C) 2021 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from the FirmataExpress project and the AGPL-3. License
 *  Copyright (C) 2006-2008 Hans-Christoph Steiner. All rights reserved.
 *  Copyright (C) 2009-2017 Jeff Hoefs. All rights reserved.
 *  Copyright (C) 2018-2019 Alan Yorinks. All Rights Reserved.
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3
 *  See DOCS/LICENSING.md for more information.
 */
#pragma once

#include <Boards.h>
#include <DHTStable.h>

namespace OASIS
{

class FirmataDHT
{
public:
  // Lifecycle functions
  void Setup(void (*loopFunc)());
  void Reset();
  void Loop();

  // DHT functions
  void EnableDHT(uint8_t digitalPin);
  void ConfigureDHT(int DHT_pin, int DHT_type);
  void SetSamplingInterval(uint8_t samplingIntervalMs);

private:
  // DHT state
  unsigned int m_numActiveDHTs = 0; // Number of DHTs attached
  uint8_t m_DHT_pinNumbers[MAX_DHTS]{};
  uint8_t m_DHT_wakeUpDelay[MAX_DHTS]{};
  uint8_t m_DHT_type[MAX_DHTS]{};
  DHTStable m_DHT; // Instance of DHTStable
  uint8_t m_nextDHT = 0; // Index into m_DHT[] for next device
  uint8_t m_currentDHT = 0; // Keeps track of which sensor is active
  unsigned int m_dhtNumLoops = 0;
  unsigned int m_dhtLoopCounter = 0;
  uint8_t m_dhtValue[4]{}; // Buffer to receive data
};

} // namespace OASIS

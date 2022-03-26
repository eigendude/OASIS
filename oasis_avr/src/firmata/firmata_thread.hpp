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

#include <stdint.h>

#include <Boards.h>

namespace OASIS
{

// Forward-declare subsystems
class FirmataAnalog;
class FirmataDiagnostics;
class FirmataDigital;

class FirmataThread
{
private:
  /*!
   * \brief Create the thread
   */
  FirmataThread();

public:
  static FirmataThread& GetInstance();

  // Lifecycle functions
  void Setup();
  void Reset();
  bool IsResetting() const { return m_isResetting; }

  // Subsystems (const)
  const FirmataAnalog* GetAnalog() const { return m_analog; }
  const FirmataDiagnostics* GetDiagnostics() const { return m_diagnostics; }
  const FirmataDigital* GetDigital() const { return m_digital; }

  // Subsystems (mutable)
  FirmataAnalog* GetAnalog() { return m_analog; }
  FirmataDiagnostics* GetDiagnostics() { return m_diagnostics; }
  FirmataDigital* GetDigital() { return m_digital; }

  // Timer functions
  void SetSamplingInterval(uint8_t samplingIntervalMs);

private:
  // Threading functions
  void Loop();

  // Static threading functions
  static void FirmataLoop();

  // Subsystem static access
  static void AnalogLoop();
  static void DiagnosticsLoop();
  static void DigitalLoop();

  // Subsystems
  FirmataAnalog* m_analog{nullptr};
  FirmataDiagnostics* m_diagnostics{nullptr};
  FirmataDigital* m_digital{nullptr};

  // Lifecycle state
  bool m_isResetting = false;

  // Timer variables
  unsigned int m_samplingIntervalMs = 19; // How often to run the main loop (in ms)
};

} // namespace OASIS

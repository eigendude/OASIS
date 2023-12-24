/*
 *  Copyright (C) 2023 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_logging.hpp"

#include "telemetrix_reports.hpp"

#include <string.h>

#include <Arduino.h>
#include <HardwareSerial.h>

using namespace OASIS;

bool TelemetrixLogging::m_loggingEnabled{false};

void TelemetrixLogging::LogString(const char* stringData)
{
  if (m_loggingEnabled)
  {
    // TODO: Why does logging too fast break the protocol? To prevent this,
    // limit individual log lines to 20 chars, and hope that we don't log too
    // many lines at once.
    const size_t stringLength = min(strlen(stringData), 22);

    Serial.write(stringLength + 1);
    Serial.write(STRING_DATA);
    for (size_t i = 0; i < stringLength; ++i)
      Serial.write(stringData[i]);
  }
}

void TelemetrixLogging::EnableLogging()
{
  m_loggingEnabled = true;
  LogString("Logging enabled");
}

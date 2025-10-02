/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See DOCS/LICENSING.md for more information.
 */

#include "telemetrix_thread.hpp"

#include "scheduler/task_scheduler.hpp"

#include <Arduino.h>

using namespace OASIS;

TelemetrixThread& TelemetrixThread::GetInstance()
{
  static TelemetrixThread instance;
  return instance;
}

void TelemetrixThread::Setup()
{
  m_server.Setup();

  InitializeTaskScheduler();

  static TsTask telemetrixCommandTask(TASK_IMMEDIATE, TASK_FOREVER, TelemetrixCommandLoop);
  static TsTask telemetrixSensorTask(TASK_IMMEDIATE, TASK_FOREVER, TelemetrixSensorLoop);

  GetTaskScheduler().addTask(telemetrixCommandTask);
  GetTaskScheduler().addTask(telemetrixSensorTask);

  telemetrixCommandTask.enable();
  telemetrixSensorTask.enable();
}

void TelemetrixThread::CommandLoop()
{
  m_server.ProcessCommands();
}

void TelemetrixThread::SensorLoop()
{
  m_server.ScanSensors();
}

void TelemetrixThread::TelemetrixCommandLoop()
{
  GetInstance().CommandLoop();
}

void TelemetrixThread::TelemetrixSensorLoop()
{
  GetInstance().SensorLoop();
}

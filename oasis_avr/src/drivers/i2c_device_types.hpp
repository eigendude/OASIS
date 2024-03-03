/*
 *  Copyright (C) 2022-2024 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  This file is derived from Telemetrix4Arduino under the AGPL 3.0 License
 *  Copyright (c) 2020-2021 Alan Yorinks
 *
 *  SPDX-License-Identifier: Apache-2.0 AND AGPL-3.0
 *  See DOCS/LICENSING.md for more information.
 */

#pragma once

#include <stdint.h>

namespace OASIS
{
enum class I2CDeviceType
{
  CCS811,
  MPU6050,
};

using CCS811ScanCallback = void (*)(uint8_t i2cPortIndex,
                                    uint8_t i2cAddress,
                                    uint16_t co2Ppb,
                                    uint16_t tvocPpb);

using MPU6050ScanCallback = void (*)(uint8_t i2cPortIndex,
                                     uint8_t i2cAddress,
                                     int16_t ax,
                                     int16_t ay,
                                     int16_t az,
                                     int16_t gx,
                                     int16_t gy,
                                     int16_t gz);
} // namespace OASIS

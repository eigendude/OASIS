/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "magnetometer/io/MagnetometerCalibrationFile.h"

#include <fstream>

#include <yaml-cpp/yaml.h>

namespace OASIS
{
namespace Magnetometer
{

bool MagnetometerCalibrationFile::Write(const std::filesystem::path& path,
                                        const MagnetometerCalibrationRecord& record) const
{
  YAML::Node root;
  root["timestamp_s"] = record.timestamp_s;
  root["bandwidth_mode"] = static_cast<unsigned int>(record.bandwidth_mode);
  root["raw_rate_hz"] = record.raw_rate_hz;

  YAML::Node covariance = YAML::Node(YAML::NodeType::Sequence);
  covariance.push_back(record.covariance_t2(0, 0));
  covariance.push_back(record.covariance_t2(0, 1));
  covariance.push_back(record.covariance_t2(0, 2));
  covariance.push_back(record.covariance_t2(1, 0));
  covariance.push_back(record.covariance_t2(1, 1));
  covariance.push_back(record.covariance_t2(1, 2));
  covariance.push_back(record.covariance_t2(2, 0));
  covariance.push_back(record.covariance_t2(2, 1));
  covariance.push_back(record.covariance_t2(2, 2));
  root["covariance_t2"] = covariance;

  YAML::Node offset = YAML::Node(YAML::NodeType::Sequence);
  offset.push_back(record.offset_t.x());
  offset.push_back(record.offset_t.y());
  offset.push_back(record.offset_t.z());
  root["offset_t"] = offset;

  std::filesystem::path tmpPath = path;
  tmpPath += ".tmp";

  std::ofstream stream(tmpPath);
  if (!stream.is_open())
    return false;

  stream << root;
  stream.close();

  std::error_code error;
  std::filesystem::rename(tmpPath, path, error);
  return !error;
}

} // namespace Magnetometer
} // namespace OASIS

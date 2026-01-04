/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/io/ImuCalibrationFile.h"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace OASIS::IMU
{
namespace
{
bool ReadSeqDoubles(const YAML::Node& node, std::size_t n, std::vector<double>& out)
{
  if (!node || !node.IsSequence() || node.size() != n)
    return false;

  out.resize(n);
  for (std::size_t i = 0; i < n; ++i)
    out[i] = node[i].as<double>();

  return true;
}

YAML::Node WriteSeqDoubles(const std::vector<double>& v)
{
  YAML::Node seq(YAML::NodeType::Sequence);
  for (double x : v)
    seq.push_back(x);
  return seq;
}

bool ReadVec3(const YAML::Node& node, Vec3& out)
{
  std::vector<double> v;
  if (!ReadSeqDoubles(node, 3, v))
    return false;

  out = {v[0], v[1], v[2]};
  return true;
}

YAML::Node WriteVec3(const Vec3& v)
{
  return WriteSeqDoubles({v[0], v[1], v[2]});
}

bool ReadMat3RowMajor(const YAML::Node& node, Mat3& out)
{
  std::vector<double> v;
  if (!ReadSeqDoubles(node, 9, v))
    return false;

  out[0][0] = v[0];
  out[0][1] = v[1];
  out[0][2] = v[2];

  out[1][0] = v[3];
  out[1][1] = v[4];
  out[1][2] = v[5];

  out[2][0] = v[6];
  out[2][1] = v[7];
  out[2][2] = v[8];

  return true;
}

YAML::Node WriteMat3RowMajor(const Mat3& m)
{
  return WriteSeqDoubles({
      m[0][0],
      m[0][1],
      m[0][2],
      m[1][0],
      m[1][1],
      m[1][2],
      m[2][0],
      m[2][1],
      m[2][2],
  });
}

bool ReadMat12RowMajor(const YAML::Node& node, std::array<std::array<double, 12>, 12>& out)
{
  std::vector<double> v;
  if (!ReadSeqDoubles(node, 144, v))
    return false;

  std::size_t k = 0;
  for (std::size_t r = 0; r < 12; ++r)
  {
    for (std::size_t c = 0; c < 12; ++c)
    {
      out[r][c] = v[k];
      ++k;
    }
  }
  return true;
}

YAML::Node WriteMat12RowMajor(const std::array<std::array<double, 12>, 12>& m)
{
  std::vector<double> v;
  v.reserve(144);

  for (std::size_t r = 0; r < 12; ++r)
  {
    for (std::size_t c = 0; c < 12; ++c)
      v.push_back(m[r][c]);
  }

  return WriteSeqDoubles(v);
}

bool ReadAccelA(const YAML::Node& node, Mat3& out)
{
  return ReadMat3RowMajor(node, out);
}

YAML::Node WriteAccelA(const Mat3& A)
{
  return WriteMat3RowMajor(A);
}

bool RequireScalar(const YAML::Node& node)
{
  return node && node.IsScalar();
}
} // namespace

bool ImuCalibrationFile::Load(const std::filesystem::path& path, ImuCalibrationRecord& out) const
{
  out = ImuCalibrationRecord{};
  out.calib.valid = false;

  if (!std::filesystem::exists(path))
    return false;

  YAML::Node root;
  try
  {
    root = YAML::LoadFile(path.string());
  }
  catch (...)
  {
    return false;
  }

  if (!RequireScalar(root["version"]))
    return false;

  const int version = root["version"].as<int>();
  if (version != Version())
    return false;

  if (!RequireScalar(root["created_unix_ns"]))
    return false;
  out.created_unix_ns = root["created_unix_ns"].as<std::uint64_t>();

  if (!RequireScalar(root["gravity_mps2"]))
    return false;
  out.gravity_mps2 = root["gravity_mps2"].as<double>();

  if (!RequireScalar(root["fit_sample_count"]))
    return false;
  out.fit_sample_count = root["fit_sample_count"].as<std::uint32_t>();

  const YAML::Node calib = root["calib"];
  if (!calib || !calib.IsMap())
    return false;

  if (!ReadVec3(calib["accel_bias_mps2"], out.calib.accel_bias_mps2))
    return false;

  if (!ReadAccelA(calib["accel_A_row_major"], out.calib.accel_A))
    return false;

  if (!ReadMat12RowMajor(calib["accel_param_cov_row_major_12x12"], out.calib.accel_param_cov))
    return false;

  if (!RequireScalar(calib["rms_residual_mps2"]))
    return false;
  out.calib.rms_residual_mps2 = calib["rms_residual_mps2"].as<double>();

  if (!ReadVec3(calib["gyro_bias_rads"], out.calib.gyro_bias_rads))
    return false;

  if (!ReadMat3RowMajor(calib["gyro_bias_cov_row_major"], out.calib.gyro_bias_cov_rads2_2))
    return false;

  if (!RequireScalar(calib["temperature_c"]))
    return false;
  out.calib.temperature_c = calib["temperature_c"].as<double>();

  if (!RequireScalar(calib["temperature_var_c2"]))
    return false;
  out.calib.temperature_var_c2 = calib["temperature_var_c2"].as<double>();

  const YAML::Node noise = root["measurement_noise"];
  if (!noise || !noise.IsMap())
    return false;

  if (!ReadMat3RowMajor(noise["accel_cov_raw_mps2_2"],
                        out.measurement_noise.accel_cov_raw_mps2_2))
    return false;

  if (!ReadMat3RowMajor(noise["gyro_cov_raw_rads2_2"],
                        out.measurement_noise.gyro_cov_raw_rads2_2))
    return false;

  if (!ReadMat3RowMajor(noise["accel_cov_corrected_mps2_2"],
                        out.measurement_noise.accel_cov_corrected_mps2_2))
    return false;

  if (!ReadMat3RowMajor(noise["gyro_cov_corrected_rads2_2"],
                        out.measurement_noise.gyro_cov_corrected_rads2_2))
    return false;

  const YAML::Node ell = root["accel_ellipsoid"];
  if (!ell || !ell.IsMap())
    return false;

  if (!ReadVec3(ell["center_mps2"], out.accel_ellipsoid.center_mps2))
    return false;

  if (!ReadMat3RowMajor(ell["Q_row_major"], out.accel_ellipsoid.Q))
    return false;

  out.calib.valid = true;
  return true;
}

bool ImuCalibrationFile::Save(const std::filesystem::path& path,
                              const ImuCalibrationRecord& rec) const
{
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec)
    return false;

  YAML::Node root;
  root["version"] = Version();
  root["created_unix_ns"] = rec.created_unix_ns;
  root["gravity_mps2"] = rec.gravity_mps2;
  root["fit_sample_count"] = rec.fit_sample_count;

  YAML::Node calib;
  calib["accel_bias_mps2"] = WriteVec3(rec.calib.accel_bias_mps2);
  calib["accel_A_row_major"] = WriteAccelA(rec.calib.accel_A);
  calib["accel_param_cov_row_major_12x12"] = WriteMat12RowMajor(rec.calib.accel_param_cov);
  calib["rms_residual_mps2"] = rec.calib.rms_residual_mps2;
  calib["gyro_bias_rads"] = WriteVec3(rec.calib.gyro_bias_rads);
  calib["gyro_bias_cov_row_major"] = WriteMat3RowMajor(rec.calib.gyro_bias_cov_rads2_2);
  calib["temperature_c"] = rec.calib.temperature_c;
  calib["temperature_var_c2"] = rec.calib.temperature_var_c2;

  root["calib"] = calib;

  YAML::Node noise;
  noise["accel_cov_raw_mps2_2"] =
      WriteMat3RowMajor(rec.measurement_noise.accel_cov_raw_mps2_2);
  noise["gyro_cov_raw_rads2_2"] =
      WriteMat3RowMajor(rec.measurement_noise.gyro_cov_raw_rads2_2);
  noise["accel_cov_corrected_mps2_2"] =
      WriteMat3RowMajor(rec.measurement_noise.accel_cov_corrected_mps2_2);
  noise["gyro_cov_corrected_rads2_2"] =
      WriteMat3RowMajor(rec.measurement_noise.gyro_cov_corrected_rads2_2);

  root["measurement_noise"] = noise;

  YAML::Node ell;
  ell["center_mps2"] = WriteVec3(rec.accel_ellipsoid.center_mps2);
  ell["Q_row_major"] = WriteMat3RowMajor(rec.accel_ellipsoid.Q);

  root["accel_ellipsoid"] = ell;

  std::ofstream f(path, std::ios::out | std::ios::trunc);
  if (!f.is_open())
    return false;

  f << root;
  f << "\n";
  return static_cast<bool>(f);
}

std::string ImuCalibrationFile::DefaultFilename()
{
  return "imu_calibration.yaml";
}
} // namespace OASIS::IMU

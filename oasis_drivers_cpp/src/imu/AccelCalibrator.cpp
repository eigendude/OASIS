/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "imu/AccelCalibrator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>

#include <yaml-cpp/yaml.h>

using namespace OASIS::IMU;

namespace
{
constexpr double kNoiseAlpha = 0.05; // Exponential smoothing for noise.
constexpr double kNoiseFloor = 1e-6; // Prevent zero variance in early boot.
constexpr double kSaveCooldownSeconds = 1.0; // Autosave guard.
constexpr double kBiasAlpha = 0.001; // Slow IIR for gyro bias removal.
constexpr double kSigmaMeanFloor = 0.002; // Floor for gyro mean sigma (rad/s).
constexpr size_t kMinClusters = 10; // Minimum clusters before attempting a fit.
constexpr double kSpreadThreshold = 0.3; // Directional spread gate for fitting.

// Utility helpers for small fixed-size vectors/matrices
std::array<double, 3> Subtract(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

std::array<double, 3> Scale(const std::array<double, 3>& a, double s)
{
  return {a[0] * s, a[1] * s, a[2] * s};
}

double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const std::array<double, 3>& a)
{
  return std::sqrt(Dot(a, a));
}

std::array<double, 3> MatrixVector(const std::array<std::array<double, 3>, 3>& A,
                                   const std::array<double, 3>& x)
{
  return {A[0][0] * x[0] + A[0][1] * x[1] + A[0][2] * x[2],
          A[1][0] * x[0] + A[1][1] * x[1] + A[1][2] * x[2],
          A[2][0] * x[0] + A[2][1] * x[1] + A[2][2] * x[2]};
}

std::array<std::array<double, 3>, 3> MultiplyATransposeA(
    const std::array<std::array<double, 3>, 3>& A)
{
  std::array<std::array<double, 3>, 3> W{};

  for (size_t r = 0; r < 3; ++r)
  {
    for (size_t c = 0; c < 3; ++c)
    {
      double value = 0.0;
      for (size_t k = 0; k < 3; ++k)
        value += A[k][r] * A[k][c];
      W[r][c] = value;
    }
  }

  return W;
}

bool SolveLinearSystem(std::vector<std::vector<double>> A,
                       std::vector<double> b,
                       std::vector<double>& x)
{
  const size_t n = b.size();

  for (size_t i = 0; i < n; ++i)
  {
    // Pivot
    size_t pivot = i;
    double max_val = std::abs(A[i][i]);
    for (size_t r = i + 1; r < n; ++r)
    {
      if (std::abs(A[r][i]) > max_val)
      {
        max_val = std::abs(A[r][i]);
        pivot = r;
      }
    }

    if (max_val < 1e-12)
      return false;

    if (pivot != i)
    {
      std::swap(A[i], A[pivot]);
      std::swap(b[i], b[pivot]);
    }

    const double diag = A[i][i];
    for (size_t c = i; c < n; ++c)
      A[i][c] /= diag;
    b[i] /= diag;

    for (size_t r = 0; r < n; ++r)
    {
      if (r == i)
        continue;
      const double factor = A[r][i];
      if (std::abs(factor) < 1e-12)
        continue;
      for (size_t c = i; c < n; ++c)
        A[r][c] -= factor * A[i][c];
      b[r] -= factor * b[i];
    }
  }

  x = b;
  return true;
}

bool InvertMatrix(const std::vector<std::vector<double>>& A,
                  std::vector<std::vector<double>>& inverse)
{
  const size_t n = A.size();
  inverse.assign(n, std::vector<double>(n, 0.0));

  for (size_t i = 0; i < n; ++i)
    inverse[i][i] = 1.0;

  std::vector<std::vector<double>> aug = A;

  for (size_t i = 0; i < n; ++i)
  {
    size_t pivot = i;
    double max_val = std::abs(aug[i][i]);
    for (size_t r = i + 1; r < n; ++r)
    {
      if (std::abs(aug[r][i]) > max_val)
      {
        max_val = std::abs(aug[r][i]);
        pivot = r;
      }
    }

    if (max_val < 1e-12)
      return false;

    if (pivot != i)
    {
      std::swap(aug[i], aug[pivot]);
      std::swap(inverse[i], inverse[pivot]);
    }

    const double diag = aug[i][i];
    for (size_t c = 0; c < n; ++c)
    {
      aug[i][c] /= diag;
      inverse[i][c] /= diag;
    }

    for (size_t r = 0; r < n; ++r)
    {
      if (r == i)
        continue;
      const double factor = aug[r][i];
      if (std::abs(factor) < 1e-12)
        continue;
      for (size_t c = 0; c < n; ++c)
      {
        aug[r][c] -= factor * aug[i][c];
        inverse[r][c] -= factor * inverse[i][c];
      }
    }
  }

  return true;
}

AccelCalibrator::WindowSample ComputeWindowStats(
    const std::vector<AccelCalibrator::WindowSample>& window)
{
  AccelCalibrator::WindowSample stats{};

  if (window.empty())
    return stats;

  const double count = static_cast<double>(window.size());

  for (const auto& sample : window)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      stats.mean_accel[i] += sample.mean_accel[i];
      stats.mean_gyro[i] += sample.mean_gyro[i];
    }
    stats.mean_norm += sample.mean_norm;
  }

  for (size_t i = 0; i < 3; ++i)
  {
    stats.mean_accel[i] /= count;
    stats.mean_gyro[i] /= count;
  }
  stats.mean_norm /= count;

  for (const auto& sample : window)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      const double da = sample.mean_accel[i] - stats.mean_accel[i];
      const double dg = sample.mean_gyro[i] - stats.mean_gyro[i];
      stats.var_accel[i] += da * da;
      stats.var_gyro[i] += dg * dg;
    }

    const double dn = sample.mean_norm - stats.mean_norm;
    stats.stddev_norm += dn * dn;
  }

  for (size_t i = 0; i < 3; ++i)
  {
    stats.var_accel[i] /= count;
    stats.var_gyro[i] /= count;
  }
  stats.stddev_norm = std::sqrt(stats.stddev_norm / count);

  return stats;
}

AccelCalibrator::Calibration ParseCalibration(const YAML::Node& root)
{
  AccelCalibrator::Calibration calib{};

  calib.version = root["version"].as<int>(1);
  calib.frame_id = root["frame_id"].as<std::string>("imu_link");
  calib.created_unix_ns = root["created_unix_ns"].as<std::uint64_t>(0);

  const auto config = root["config"];
  if (config)
    calib.gravity_mps2 = config["gravity"].as<double>(calib.gravity_mps2);

  const auto noise = root["noise"];
  if (noise)
  {
    auto accel = noise["accel_stddev_mps2"];
    auto gyro = noise["gyro_stddev_rads"];
    for (size_t i = 0; i < 3; ++i)
    {
      calib.accel_noise_stddev_mps2[i] = accel ? accel[i].as<double>(0.0) : 0.0;
      calib.gyro_noise_stddev_rads[i] = gyro ? gyro[i].as<double>(0.0) : 0.0;
    }
  }

  const auto accel = root["accel_calibration"];
  if (accel)
  {
    auto bias = accel["bias_mps2"];
    for (size_t i = 0; i < 3; ++i)
      calib.bias_mps2[i] = bias ? bias[i].as<double>(0.0) : 0.0;

    auto A = accel["A"];
    if (A)
    {
      for (size_t r = 0; r < 3; ++r)
      {
        auto row = A[r];
        for (size_t c = 0; c < 3; ++c)
          calib.A[r][c] = row ? row[c].as<double>(r == c ? 1.0 : 0.0) : 0.0;
      }
    }

    auto bias_stddev = accel["bias_stddev_mps2"];
    for (size_t i = 0; i < 3; ++i)
      calib.bias_stddev_mps2[i] = bias_stddev ? bias_stddev[i].as<double>(0.0) : 0.0;

    auto A_stddev = accel["A_stddev"];
    if (A_stddev)
    {
      for (size_t r = 0; r < 3; ++r)
      {
        auto row = A_stddev[r];
        for (size_t c = 0; c < 3; ++c)
          calib.A_stddev[r][c] = row ? row[c].as<double>(0.0) : 0.0;
      }
    }
  }

  const auto ellipsoid = root["ellipsoid"];
  if (ellipsoid)
  {
    auto W = ellipsoid["W"];
    if (W)
    {
      for (size_t r = 0; r < 3; ++r)
      {
        auto row = W[r];
        for (size_t c = 0; c < 3; ++c)
          calib.W[r][c] = row ? row[c].as<double>(r == c ? 1.0 : 0.0) : 0.0;
      }
    }
    auto center = ellipsoid["center"];
    for (size_t i = 0; i < 3; ++i)
      calib.center_mps2[i] = center ? center[i].as<double>(0.0) : 0.0;
  }

  const auto fit_quality = root["fit_quality"];
  if (fit_quality)
  {
    calib.rms_residual_mps2 = fit_quality["rms_residual_mps2"].as<double>(0.0);
    calib.num_clusters = fit_quality["num_clusters"].as<size_t>(0);
    calib.num_samples = fit_quality["num_samples"].as<size_t>(0);
  }

  const auto temperature = root["temperature"];
  if (temperature)
  {
    calib.temperature_mean_c = temperature["mean_c"].as<double>(0.0);
    calib.temperature_stddev_c = temperature["stddev_c"].as<double>(0.0);
  }

  return calib;
}

YAML::Node ToSequence(const std::array<double, 3>& values)
{
  YAML::Node node;
  for (double value : values)
    node.push_back(value);
  return node;
}

YAML::Node ToMatrix(const std::array<std::array<double, 3>, 3>& values)
{
  YAML::Node node;
  for (const auto& row_values : values)
  {
    YAML::Node row;
    for (double value : row_values)
      row.push_back(value);
    node.push_back(row);
  }
  return node;
}

YAML::Node SerializeCalibration(const AccelCalibrator::Calibration& calib)
{
  YAML::Node root;
  root["version"] = calib.version;
  root["frame_id"] = calib.frame_id;
  root["created_unix_ns"] = calib.created_unix_ns;

  YAML::Node config;
  config["gravity"] = calib.gravity_mps2;
  root["config"] = config;

  YAML::Node temperature;
  temperature["mean_c"] = calib.temperature_mean_c;
  temperature["stddev_c"] = calib.temperature_stddev_c;
  root["temperature"] = temperature;

  YAML::Node noise;
  noise["accel_stddev_mps2"] = ToSequence(calib.accel_noise_stddev_mps2);
  noise["gyro_stddev_rads"] = ToSequence(calib.gyro_noise_stddev_rads);
  root["noise"] = noise;

  YAML::Node accel;
  accel["bias_mps2"] = ToSequence(calib.bias_mps2);
  accel["A"] = ToMatrix(calib.A);
  accel["bias_stddev_mps2"] = ToSequence(calib.bias_stddev_mps2);
  accel["A_stddev"] = ToMatrix(calib.A_stddev);
  root["accel_calibration"] = accel;

  YAML::Node ellipsoid;
  ellipsoid["W"] = ToMatrix(calib.W);
  ellipsoid["center"] = ToSequence(calib.center_mps2);
  root["ellipsoid"] = ellipsoid;

  YAML::Node fit_quality;
  fit_quality["rms_residual_mps2"] = calib.rms_residual_mps2;
  fit_quality["num_clusters"] = calib.num_clusters;
  fit_quality["num_samples"] = calib.num_samples;
  root["fit_quality"] = fit_quality;

  return root;
}
} // namespace

void AccelCalibrator::Configure(const Config& config,
                                const std::filesystem::path& cache_path,
                                const std::string& frame_id)
{
  m_config = config;
  m_cache_path = cache_path;
  m_frame_id = frame_id;
}

void AccelCalibrator::Reset()
{
  m_noise_stddev_accel = {0.0, 0.0, 0.0};
  m_noise_stddev_gyro = {0.0, 0.0, 0.0};
  m_noise_initialized = false;
  m_window.clear();
  m_clusters.clear();
  m_total_pose_samples = 0;
  m_calibration.reset();
  m_dirty = false;
  m_last_save_time_s = 0.0;
  m_gyro_bias_iir = {0.0, 0.0, 0.0};
  m_gyro_bias_iir_init = false;
}

bool AccelCalibrator::LoadCache()
{
  if (m_cache_path.empty() || !std::filesystem::exists(m_cache_path))
    return false;

  try
  {
    YAML::Node root = YAML::LoadFile(m_cache_path.string());
    AccelCalibrator::Calibration calib = ParseCalibration(root);

    if (calib.frame_id != m_frame_id || calib.version != 1)
      return false;

    m_calibration = calib;
    m_config.gravity_mps2 = calib.gravity_mps2;
    m_calibration_mode = false;
    m_noise_stddev_accel = calib.accel_noise_stddev_mps2;
    m_noise_stddev_gyro = calib.gyro_noise_stddev_rads;
    m_noise_initialized = true;
    return true;
  }
  catch (const YAML::Exception&)
  {
    return false;
  }
}

std::array<double, 3> AccelCalibrator::Apply(const std::array<double, 3>& accel_mps2) const
{
  if (!m_calibration)
    return accel_mps2;

  const std::array<double, 3> centered = Subtract(accel_mps2, m_calibration->bias_mps2);
  return MatrixVector(m_calibration->A, centered);
}

AccelCalibrator::UpdateStatus AccelCalibrator::Update(const Sample& sample)
{
  UpdateStatus status{};

  // Update noise tracking first for use by the stationary detector.
  UpdateNoiseEstimates(sample);

  // Append to the sliding window.
  WindowSample window_entry{};
  window_entry.mean_accel = sample.accel_mps2;
  window_entry.mean_gyro = sample.gyro_rads;
  window_entry.mean_norm = Norm(sample.accel_mps2);
  m_window.push_back(window_entry);
  if (m_window.size() > m_config.window_size)
    m_window.erase(m_window.begin());

  const WindowSample stats = ComputeWindowStats(m_window);
  status.stationary = DetectStationary(sample, stats);

  if (m_calibration_mode && status.stationary)
    MergePose(sample, stats);

  status.num_clusters = m_clusters.size();
  status.num_pose_samples = m_total_pose_samples;
  status.has_axis_coverage = HasAxisCoverage();

  if (m_calibration_mode && status.stationary)
  {
    const size_t param_count = 9;
    const size_t required_clusters = std::max(kMinClusters, param_count + static_cast<size_t>(6));
    const bool enough_clusters = m_clusters.size() >= required_clusters;
    const double spread = ComputeDirectionalSpread();
    const bool has_spread = spread > kSpreadThreshold;

    if (enough_clusters && has_spread)
    {
      status.has_solution = FitEllipsoid();
      if (status.has_solution)
        status.wrote_cache = SaveCache(sample);
    }
  }
  else if (m_calibration)
  {
    status.has_solution = true;
  }

  return status;
}

void AccelCalibrator::UpdateNoiseEstimates(const Sample& sample)
{
  auto update_axis = [](double current, double measurement)
  {
    if (current <= 0.0)
      return std::max(measurement, kNoiseFloor);
    return (1.0 - kNoiseAlpha) * current + kNoiseAlpha * measurement;
  };

  for (size_t axis = 0; axis < 3; ++axis)
  {
    const double accel_sigma = std::sqrt(std::max(sample.accel_var_mps2_2[axis], kNoiseFloor));
    const double gyro_sigma = std::sqrt(std::max(sample.gyro_var_rads2_2[axis], kNoiseFloor));
    m_noise_stddev_accel[axis] = update_axis(m_noise_stddev_accel[axis], accel_sigma);
    m_noise_stddev_gyro[axis] = update_axis(m_noise_stddev_gyro[axis], gyro_sigma);
  }

  m_noise_initialized = true;
}

bool AccelCalibrator::DetectStationary(const Sample& sample, const WindowSample& stats)
{
  (void)sample;

  if (m_window.size() < m_config.window_size / 2)
    return false;

  // Stage A: variance-only stillness gates.
  for (size_t axis = 0; axis < 3; ++axis)
  {
    const double accel_var_noise =
        std::max(m_noise_stddev_accel[axis] * m_noise_stddev_accel[axis], kNoiseFloor);
    if (stats.var_accel[axis] > accel_var_noise * m_config.variance_threshold)
      return false;

    const double gyro_var_noise =
        std::max(m_noise_stddev_gyro[axis] * m_noise_stddev_gyro[axis], kNoiseFloor);
    if (stats.var_gyro[axis] > gyro_var_noise * m_config.variance_threshold)
      return false;
  }

  const double noise_mag_sigma = std::sqrt(Dot(m_noise_stddev_accel, m_noise_stddev_accel));
  if (stats.stddev_norm > m_config.variance_threshold * noise_mag_sigma)
    return false;

  // Sanity check: loose magnitude check to reject gross outliers.
  const double mean_norm = stats.mean_norm;
  if (std::abs(mean_norm - m_config.gravity_mps2) > m_config.gravity_mps2 * 0.5)
    return false;

  // Stage B: mean gyro gate using a slowly de-biased gyro mean.
  if (!m_gyro_bias_iir_init)
  {
    m_gyro_bias_iir = stats.mean_gyro;
    m_gyro_bias_iir_init = true;
  }
  else
  {
    for (size_t axis = 0; axis < 3; ++axis)
    {
      m_gyro_bias_iir[axis] =
          (1.0 - kBiasAlpha) * m_gyro_bias_iir[axis] + kBiasAlpha * stats.mean_gyro[axis];
    }
  }

  const double n = static_cast<double>(m_window.size());
  for (size_t axis = 0; axis < 3; ++axis)
  {
    const double noise_sigma = std::max(m_noise_stddev_gyro[axis], std::sqrt(kNoiseFloor));
    const double sigma_mean = std::max(noise_sigma / std::sqrt(std::max(1.0, n)), kSigmaMeanFloor);
    const double mean_gyro_unbiased = stats.mean_gyro[axis] - m_gyro_bias_iir[axis];
    if (std::abs(mean_gyro_unbiased) > m_config.gyro_mean_threshold * sigma_mean)
      return false;
  }

  return true;
}

void AccelCalibrator::MergePose(const Sample& sample, const WindowSample& stats)
{
  std::array<double, 3> pose_mean = stats.mean_accel;
  std::array<double, 3> pose_cov{0.0, 0.0, 0.0};
  for (size_t axis = 0; axis < 3; ++axis)
  {
    const double sigma2 =
        std::max(m_noise_stddev_accel[axis] * m_noise_stddev_accel[axis], kNoiseFloor);
    pose_cov[axis] = sigma2 / std::max<size_t>(1, m_window.size());
  }

  const double merge_sigma = m_config.merge_sigma;
  const double merge_threshold = 3.0 * merge_sigma * merge_sigma;
  size_t best_cluster = m_clusters.size();
  double best_distance = merge_threshold;

  for (size_t i = 0; i < m_clusters.size(); ++i)
  {
    const auto& cluster = m_clusters[i];
    double d2 = 0.0;
    for (size_t axis = 0; axis < 3; ++axis)
    {
      const double sigma = merge_sigma * m_noise_stddev_accel[axis];
      const double sigma2 = std::max(sigma * sigma, kNoiseFloor);
      const double diff = pose_mean[axis] - cluster.mean[axis];
      d2 += (diff * diff) / sigma2;
    }
    if (d2 < best_distance)
    {
      best_distance = d2;
      best_cluster = i;
    }
  }

  if (best_cluster == m_clusters.size())
  {
    Cluster cluster{};
    cluster.mean = pose_mean;
    cluster.covariance = pose_cov;
    cluster.weight = 1.0;
    cluster.count = 1;
    cluster.temperature_mean_c = sample.temperature_c;
    m_clusters.emplace_back(cluster);
  }
  else
  {
    Cluster& cluster = m_clusters[best_cluster];
    for (size_t axis = 0; axis < 3; ++axis)
    {
      const double cov_old = std::max(cluster.covariance[axis], kNoiseFloor);
      const double cov_new = std::max(pose_cov[axis], kNoiseFloor);
      const double inv_cov_old = 1.0 / cov_old;
      const double inv_cov_new = 1.0 / cov_new;
      const double cov_merged = 1.0 / (inv_cov_old + inv_cov_new);
      const double mean_merged =
          cov_merged * (cluster.mean[axis] * inv_cov_old + pose_mean[axis] * inv_cov_new);
      cluster.mean[axis] = mean_merged;
      cluster.covariance[axis] = cov_merged;
    }
    cluster.weight += 1.0;
    cluster.count += 1;
    const double delta_temp = sample.temperature_c - cluster.temperature_mean_c;
    cluster.temperature_mean_c += delta_temp / static_cast<double>(cluster.count);
  }

  ++m_total_pose_samples;
  m_dirty = true;
}

bool AccelCalibrator::HasAxisCoverage() const
{
  if (m_clusters.size() < 4)
    return false;

  const std::array<std::array<double, 3>, 6> axes{{{1.0, 0.0, 0.0},
                                                   {-1.0, 0.0, 0.0},
                                                   {0.0, 1.0, 0.0},
                                                   {0.0, -1.0, 0.0},
                                                   {0.0, 0.0, 1.0},
                                                   {0.0, 0.0, -1.0}}};
  const double cos_max_angle = std::cos(m_config.max_axis_angle_rad);
  std::array<bool, 6> covered{false, false, false, false, false, false};

  for (const auto& cluster : m_clusters)
  {
    const double norm = Norm(cluster.mean);
    if (norm < 1e-6)
      continue;
    const std::array<double, 3> u = Scale(cluster.mean, 1.0 / norm);
    for (size_t i = 0; i < axes.size(); ++i)
    {
      if (covered[i])
        continue;
      const double dot = Dot(u, axes[i]);
      if (dot > cos_max_angle)
        covered[i] = true;
    }
  }

  return std::all_of(covered.begin(), covered.end(), [](bool v) { return v; });
}

double AccelCalibrator::ComputeDirectionalSpread() const
{
  std::array<std::array<double, 3>, 3> cov{};
  size_t count = 0;

  for (const auto& cluster : m_clusters)
  {
    const double norm = Norm(cluster.mean);
    if (norm < 1e-6)
      continue;

    const std::array<double, 3> u = Scale(cluster.mean, 1.0 / norm);
    for (size_t r = 0; r < 3; ++r)
    {
      for (size_t c = 0; c < 3; ++c)
        cov[r][c] += u[r] * u[c];
    }
    ++count;
  }

  if (count == 0)
    return 0.0;

  for (auto& row : cov)
  {
    for (double& v : row)
      v /= static_cast<double>(count);
  }

  const double trace = cov[0][0] + cov[1][1] + cov[2][2];
  const double maxdiag = std::max({cov[0][0], cov[1][1], cov[2][2]});
  return trace - maxdiag;
}

bool AccelCalibrator::FitEllipsoid()
{
  if (m_clusters.size() < 4)
    return false;

  // Parameter vector: [bx, by, bz, a00, a10, a11, a20, a21, a22]
  std::array<double, 9> params{0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0};

  auto build_matrix = [&]()
  {
    std::array<std::array<double, 3>, 3> A{
        {{params[3], 0.0, 0.0}, {params[4], params[5], 0.0}, {params[6], params[7], params[8]}}};
    return A;
  };

  auto evaluate = [&](const std::array<double, 3>& vec, std::array<double, 3>& y, double& norm_y)
  {
    const std::array<double, 3> diff{vec[0] - params[0], vec[1] - params[1], vec[2] - params[2]};
    const auto A = build_matrix();
    y = MatrixVector(A, diff);
    norm_y = Norm(y);
  };

  // Gauss-Newton iterations
  const size_t max_iter = 10;
  for (size_t iter = 0; iter < max_iter; ++iter)
  {
    std::vector<double> residuals;
    residuals.reserve(m_clusters.size());
    std::vector<std::vector<double>> J;
    J.reserve(m_clusters.size());

    for (const auto& cluster : m_clusters)
    {
      std::array<double, 3> y{};
      double norm_y = 0.0;
      evaluate(cluster.mean, y, norm_y);
      if (norm_y < 1e-6)
        continue;
      const double r = norm_y - m_config.gravity_mps2;
      residuals.push_back(r);

      // Jacobian row
      std::vector<double> row(9, 0.0);
      const std::array<std::array<double, 3>, 3> A = build_matrix();
      const std::array<double, 3> unit = Scale(y, 1.0 / norm_y);

      // Bias derivatives: d/d b_j = -(A column j) dot unit
      row[0] = -(A[0][0] * unit[0] + A[1][0] * unit[1] + A[2][0] * unit[2]);
      row[1] = -(A[0][1] * unit[0] + A[1][1] * unit[1] + A[2][1] * unit[2]);
      row[2] = -(A[0][2] * unit[0] + A[1][2] * unit[1] + A[2][2] * unit[2]);

      const std::array<double, 3> diff{cluster.mean[0] - params[0], cluster.mean[1] - params[1],
                                       cluster.mean[2] - params[2]};

      // Lower-triangular A entries
      row[3] = unit[0] * diff[0]; // a00
      row[4] = unit[1] * diff[0]; // a10
      row[5] = unit[1] * diff[1]; // a11
      row[6] = unit[2] * diff[0]; // a20
      row[7] = unit[2] * diff[1]; // a21
      row[8] = unit[2] * diff[2]; // a22

      J.emplace_back(row);
    }

    if (residuals.size() < 4)
      break;

    const size_t m = residuals.size();
    const size_t n = params.size();
    std::vector<std::vector<double>> JtJ(n, std::vector<double>(n, 0.0));
    std::vector<double> Jtr(n, 0.0);

    for (size_t i = 0; i < m; ++i)
    {
      const auto& row = J[i];
      const double r = residuals[i];
      for (size_t c = 0; c < n; ++c)
      {
        Jtr[c] += row[c] * r;
        for (size_t k = 0; k < n; ++k)
          JtJ[c][k] += row[c] * row[k];
      }
    }

    std::vector<double> delta;
    if (!SolveLinearSystem(JtJ, Jtr, delta))
      break;

    const double step_norm =
        std::sqrt(std::inner_product(delta.begin(), delta.end(), delta.begin(), 0.0));
    for (size_t i = 0; i < n; ++i)
      params[i] -= delta[i];

    if (step_norm < 1e-6)
      break;
  }

  const std::array<std::array<double, 3>, 3> A = {
      {{params[3], 0.0, 0.0}, {params[4], params[5], 0.0}, {params[6], params[7], params[8]}}};
  const std::array<double, 3> bias{params[0], params[1], params[2]};

  // Final residuals and covariance
  std::vector<std::vector<double>> J_final;
  std::vector<double> residuals;
  for (const auto& cluster : m_clusters)
  {
    const std::array<double, 3> diff = Subtract(cluster.mean, bias);
    const auto y = MatrixVector(A, diff);
    const double norm_y = Norm(y);
    if (norm_y < 1e-6)
      continue;
    residuals.push_back(norm_y - m_config.gravity_mps2);

    std::vector<double> row(9, 0.0);
    const std::array<double, 3> unit = Scale(y, 1.0 / norm_y);
    row[0] = -(A[0][0] * unit[0] + A[1][0] * unit[1] + A[2][0] * unit[2]);
    row[1] = -(A[0][1] * unit[0] + A[1][1] * unit[1] + A[2][1] * unit[2]);
    row[2] = -(A[0][2] * unit[0] + A[1][2] * unit[1] + A[2][2] * unit[2]);
    row[3] = unit[0] * diff[0];
    row[4] = unit[1] * diff[0];
    row[5] = unit[1] * diff[1];
    row[6] = unit[2] * diff[0];
    row[7] = unit[2] * diff[1];
    row[8] = unit[2] * diff[2];
    J_final.emplace_back(row);
  }

  if (residuals.size() < 4)
    return false;

  const size_t m = residuals.size();
  const size_t n = params.size();
  std::vector<std::vector<double>> JtJ(n, std::vector<double>(n, 0.0));
  std::vector<double> Jtr(n, 0.0);

  double residual_energy = 0.0;
  for (size_t i = 0; i < m; ++i)
  {
    const auto& row = J_final[i];
    const double r = residuals[i];
    residual_energy += r * r;
    for (size_t c = 0; c < n; ++c)
    {
      Jtr[c] += row[c] * r;
      for (size_t k = 0; k < n; ++k)
        JtJ[c][k] += row[c] * row[k];
    }
  }

  std::vector<std::vector<double>> cov;
  if (!InvertMatrix(JtJ, cov))
    cov.assign(n, std::vector<double>(n, 0.0));

  const double dof =
      static_cast<double>(std::max<int>(1, static_cast<int>(m) - static_cast<int>(n)));
  const double sigma_r2 = residual_energy / dof;

  Calibration calib = MakeIdentityCalibration(m_config.gravity_mps2, m_frame_id);
  calib.bias_mps2 = bias;
  calib.A = A;
  calib.W = MultiplyATransposeA(A);
  calib.center_mps2 = bias;

  calib.accel_noise_stddev_mps2 = m_noise_stddev_accel;
  calib.gyro_noise_stddev_rads = m_noise_stddev_gyro;

  calib.num_clusters = m_clusters.size();
  calib.num_samples = m_total_pose_samples;
  calib.rms_residual_mps2 = std::sqrt(residual_energy / static_cast<double>(m));

  if (cov.size() == n)
  {
    calib.bias_stddev_mps2 = {std::sqrt(std::max(cov[0][0] * sigma_r2, kNoiseFloor)),
                              std::sqrt(std::max(cov[1][1] * sigma_r2, kNoiseFloor)),
                              std::sqrt(std::max(cov[2][2] * sigma_r2, kNoiseFloor))};

    calib.A_stddev = {{{std::sqrt(std::max(cov[3][3] * sigma_r2, kNoiseFloor)), 0.0, 0.0},
                       {std::sqrt(std::max(cov[4][4] * sigma_r2, kNoiseFloor)),
                        std::sqrt(std::max(cov[5][5] * sigma_r2, kNoiseFloor)), 0.0},
                       {std::sqrt(std::max(cov[6][6] * sigma_r2, kNoiseFloor)),
                        std::sqrt(std::max(cov[7][7] * sigma_r2, kNoiseFloor)),
                        std::sqrt(std::max(cov[8][8] * sigma_r2, kNoiseFloor))}}};
  }

  double temp_mean = 0.0;
  double temp_var = 0.0;
  size_t temp_count = 0;
  for (const auto& cluster : m_clusters)
  {
    temp_mean += cluster.temperature_mean_c * static_cast<double>(cluster.count);
    temp_count += cluster.count;
  }
  if (temp_count > 0)
    temp_mean /= static_cast<double>(temp_count);
  for (const auto& cluster : m_clusters)
  {
    const double diff = cluster.temperature_mean_c - temp_mean;
    temp_var += diff * diff * static_cast<double>(cluster.count);
  }
  if (temp_count > 0)
    temp_var /= static_cast<double>(temp_count);

  calib.temperature_mean_c = temp_mean;
  calib.temperature_stddev_c = std::sqrt(temp_var);

  calib.created_unix_ns =
      static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());

  m_calibration = calib;
  return true;
}

bool AccelCalibrator::SaveCache(const Sample& sample)
{
  if (!m_calibration || !m_dirty)
    return false;

  const double elapsed = sample.timestamp_s - m_last_save_time_s;
  if (m_last_save_time_s > 0.0 && elapsed < kSaveCooldownSeconds)
    return false;

  if (!m_cache_path.empty())
  {
    const std::filesystem::path cache_dir = m_cache_path.parent_path();
    if (!cache_dir.empty())
      std::filesystem::create_directories(cache_dir);
  }

  try
  {
    YAML::Node node = SerializeCalibration(*m_calibration);

    std::ofstream fout(m_cache_path, std::ios::out | std::ios::trunc);
    fout << node;
    fout.close();

    m_last_save_time_s = sample.timestamp_s;
    m_dirty = false;
    return true;
  }
  catch (const YAML::Exception&)
  {
    return false;
  }
}

AccelCalibrator::Calibration AccelCalibrator::MakeIdentityCalibration(double gravity_mps2,
                                                                      const std::string& frame_id)
{
  Calibration calib{};
  calib.frame_id = frame_id;
  calib.gravity_mps2 = gravity_mps2;
  calib.rms_residual_mps2 = 0.0;
  return calib;
}

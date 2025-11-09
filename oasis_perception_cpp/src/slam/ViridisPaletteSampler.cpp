/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "ViridisPaletteSampler.h"

#include <algorithm>
#include <array>
#include <limits>

using namespace OASIS;
using namespace SLAM;

namespace
{
struct ViridisSample
{
  float position = 0.0F;
  Eigen::Vector3f color = Eigen::Vector3f::Zero();
};

const std::array<ViridisSample, 8>& GetSamples()
{
  static const std::array<ViridisSample, 8> samples = {
      ViridisSample{0.0F, Eigen::Vector3f(0.267004F, 0.004874F, 0.329415F)},
      ViridisSample{0.125F, Eigen::Vector3f(0.282327F, 0.094955F, 0.417331F)},
      ViridisSample{0.25F, Eigen::Vector3f(0.252899F, 0.358853F, 0.594714F)},
      ViridisSample{0.375F, Eigen::Vector3f(0.211718F, 0.553018F, 0.751428F)},
      ViridisSample{0.5F, Eigen::Vector3f(0.164924F, 0.7173F, 0.607793F)},
      ViridisSample{0.625F, Eigen::Vector3f(0.134692F, 0.827384F, 0.467008F)},
      ViridisSample{0.75F, Eigen::Vector3f(0.369214F, 0.892281F, 0.273006F)},
      ViridisSample{1.0F, Eigen::Vector3f(0.993248F, 0.906157F, 0.143936F)},
  };

  return samples;
}
} // namespace

Eigen::Vector3f ViridisPaletteSampler::Sample(float t)
{
  t = std::clamp(t, 0.0F, 1.0F);

  const auto& samples = GetSamples();
  for (std::size_t index = 1; index < samples.size(); ++index)
  {
    const ViridisSample& prev = samples[index - 1];
    const ViridisSample& next = samples[index];
    if (t <= next.position)
    {
      const float span =
          std::max(next.position - prev.position, std::numeric_limits<float>::epsilon());
      const float alpha = std::clamp((t - prev.position) / span, 0.0F, 1.0F);
      return prev.color + alpha * (next.color - prev.color);
    }
  }

  return samples.back().color;
}


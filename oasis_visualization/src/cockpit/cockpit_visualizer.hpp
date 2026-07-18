/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>

namespace OASIS::Visualization
{
/** Configuration contract for the fixed Falcon cockpit display */
struct CockpitVisualizerConfig
{
  /** Output width in pixels; the installed panel requires 128 */
  int width;

  /** Output height in pixels; the installed panel requires 32 */
  int height;

  /** Preferred fraction of illuminated pixels in the range (0, 1] */
  double target_lit_fraction;

  /** Inclusive lower illuminated-pixel fraction in the range (0, 1] */
  double minimum_lit_fraction;

  /** Inclusive upper illuminated-pixel fraction in the range (0, 1] */
  double maximum_lit_fraction;

  /** Seed selecting the deterministic subsystem activity sequence */
  std::uint32_t random_seed;
};

/** Renders a deterministic, binary Falcon cockpit instrumentation panel */
class CockpitVisualizer
{
public:
  explicit CockpitVisualizer(CockpitVisualizerConfig config);

  /** Render the panel for nonnegative steady-clock elapsed time in seconds */
  cv::Mat Render(double elapsed_seconds) const;

private:
  CockpitVisualizerConfig config_;
  cv::Mat structure_;
};
} // namespace OASIS::Visualization

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "cockpit/cockpit_visualizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace OASIS::Visualization
{
namespace
{
constexpr int DISPLAY_WIDTH = 128;
constexpr int DISPLAY_HEIGHT = 32;
constexpr int DOOR_EVENT_SECONDS = 20;

std::uint64_t Mix(std::uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31U);
}

void DrawLeftConsoleBackground(cv::Mat& image)
{
  // Mechanical scan track with bold end stops and directional markers
  cv::rectangle(image, {5, 10, 32, 8}, 255, 2, cv::LINE_8);
  cv::rectangle(image, {5, 9, 3, 10}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(image, {34, 9, 3, 10}, 255, cv::FILLED, cv::LINE_8);
  const std::array<cv::Point, 3> leftArrow{{{9, 6}, {14, 3}, {14, 9}}};
  const std::array<cv::Point, 3> rightArrow{{{32, 6}, {27, 3}, {27, 9}}};
  cv::fillConvexPoly(image, leftArrow, 255, cv::LINE_8);
  cv::fillConvexPoly(image, rightArrow, 255, cv::LINE_8);

  // Large quiet status blocks maintain brightness without fine filler texture
  cv::rectangle(image, {5, 21, 8, 6}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(image, {16, 21, 8, 6}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(image, {28, 21, 8, 6}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(image, {17, 4, 7, 4}, 255, 1, cv::LINE_8);
}

void DrawDoorwayBackground(cv::Mat& image)
{
  // Stepped pressure-door arch surrounding a 28-pixel-wide black opening
  const std::array<cv::Point, 8> outerArch{{
      {44, 31},
      {44, 12},
      {48, 12},
      {48, 7},
      {54, 7},
      {54, 3},
      {75, 3},
      {75, 7},
  }};
  const std::array<cv::Point, 8> mirroredArch{{
      {75, 7},
      {81, 7},
      {81, 12},
      {85, 12},
      {85, 31},
      {81, 31},
      {81, 13},
      {77, 13},
  }};
  cv::polylines(image, outerArch, false, 255, 3, cv::LINE_8);
  cv::polylines(image, mirroredArch, false, 255, 3, cv::LINE_8);

  const std::array<cv::Point, 6> innerLeft{{
      {50, 31},
      {50, 13},
      {53, 13},
      {53, 9},
      {58, 9},
      {58, 6},
  }};
  const std::array<cv::Point, 6> innerRight{{
      {71, 6},
      {71, 9},
      {76, 9},
      {76, 13},
      {79, 13},
      {79, 31},
  }};
  cv::polylines(image, innerLeft, false, 255, 2, cv::LINE_8);
  cv::polylines(image, innerRight, false, 255, 2, cv::LINE_8);
  cv::line(image, {58, 5}, {71, 5}, 255, 2, cv::LINE_8);
  cv::line(image, {45, 30}, {51, 30}, 255, 2, cv::LINE_8);
  cv::line(image, {78, 30}, {84, 30}, 255, 2, cv::LINE_8);

  // Frame-mounted lamps never intrude into the doorway interior
  cv::rectangle(image, {62, 1, 6, 3}, 255, 1, cv::LINE_8);
}

void DrawRightConsoleBackground(cv::Mat& image)
{
  cv::rectangle(image, {91, 8, 13, 19}, 255, 2, cv::LINE_8);
  cv::rectangle(image, {108, 8, 13, 19}, 255, 2, cv::LINE_8);
  for (int y = 11; y <= 23; y += 4)
  {
    cv::line(image, {92, y}, {102, y}, 255, 1, cv::LINE_8);
    cv::line(image, {109, y}, {119, y}, 255, 1, cv::LINE_8);
  }

  const std::array<cv::Point, 4> chevron{{{96, 6}, {106, 1}, {116, 6}, {106, 4}}};
  cv::polylines(image, chevron, true, 255, 2, cv::LINE_8);
  cv::rectangle(image, {88, 3, 7, 5}, 255, 1, cv::LINE_8);
  cv::rectangle(image, {117, 3, 7, 5}, 255, 1, cv::LINE_8);
}

void DrawCriticalOutlines(cv::Mat& image)
{
  DrawLeftConsoleBackground(image);
  DrawDoorwayBackground(image);
  DrawRightConsoleBackground(image);
}

int TrianglePosition(std::uint64_t tick, int travel)
{
  const int phase = static_cast<int>(tick % static_cast<std::uint64_t>(2 * travel));
  return phase <= travel ? phase : 2 * travel - phase;
}

void DrawLeftConsoleActivity(cv::Mat& image, double elapsedSeconds, std::uint32_t seed)
{
  const std::uint64_t tick = static_cast<std::uint64_t>(elapsedSeconds * 8.0);
  const bool doorEvent = static_cast<int>(elapsedSeconds) % DOOR_EVENT_SECONDS == 16;
  const int sliderX = doorEvent ? 18 : 8 + TrianglePosition(tick, 21);
  cv::rectangle(image, {sliderX, 12, 7, 4}, 255, cv::FILLED, cv::LINE_8);

  const std::uint64_t lampTick = static_cast<std::uint64_t>(elapsedSeconds * 2.0);
  const bool alternate = (Mix(seed ^ lampTick) & 1U) != 0U;
  cv::rectangle(image, {alternate ? 7 : 29, 4, 5, 4}, 255, cv::FILLED, cv::LINE_8);
}

void DrawColumnSequence(cv::Mat& image, int x, int leader, bool synchronizedPulse)
{
  constexpr std::array<int, 5> SEGMENT_Y{{24, 20, 16, 12, 9}};
  constexpr std::array<int, 5> SEGMENT_HEIGHT{{3, 3, 3, 3, 2}};
  for (int level = 0; level < static_cast<int>(SEGMENT_Y.size()); ++level)
  {
    if (synchronizedPulse)
    {
      cv::rectangle(image, {x, SEGMENT_Y[level], 9, SEGMENT_HEIGHT[level]}, 255, cv::FILLED,
                    cv::LINE_8);
    }
    else if (level == leader)
    {
      const int leaderHeight = std::min(2, SEGMENT_HEIGHT[level]);
      cv::rectangle(image, {x, SEGMENT_Y[level], 9, leaderHeight}, 255, cv::FILLED, cv::LINE_8);
    }
    else if (level < leader)
    {
      // A one-row trail leaves the full-height leader visually distinct
      cv::rectangle(image, {x, SEGMENT_Y[level] + SEGMENT_HEIGHT[level] - 1, 9, 1}, 255, cv::FILLED,
                    cv::LINE_8);
    }
  }
}

void DrawRightConsoleActivity(cv::Mat& image, double elapsedSeconds, std::uint32_t seed)
{
  const std::uint64_t motionTick = static_cast<std::uint64_t>(elapsedSeconds * 5.0);
  const int primaryLeader = std::min(TrianglePosition(motionTick, 5), 4);
  const std::uint64_t delayedTick = motionTick == 0U ? 0U : motionTick - 1U;
  int secondaryLeader = std::min(TrianglePosition(delayedTick, 5), 4);
  if (primaryLeader == 4)
    secondaryLeader = 4;

  const bool synchronizedPulse = motionTick % 20U == 15U;
  const bool swapColumns = (seed & 1U) == 0U;
  DrawColumnSequence(image, 93, swapColumns ? secondaryLeader : primaryLeader, synchronizedPulse);
  DrawColumnSequence(image, 110, swapColumns ? primaryLeader : secondaryLeader, synchronizedPulse);

  const std::uint64_t capTick = static_cast<std::uint64_t>(elapsedSeconds * 2.0);
  const bool leftCap = ((capTick + (seed & 1U)) % 2U) == 0U;
  cv::rectangle(image, {leftCap ? 89 : 118, 4, 5, 3}, 255, cv::FILLED, cv::LINE_8);

  const int warningPhase = static_cast<int>(elapsedSeconds) % 7;
  if (warningPhase == 0)
  {
    const std::array<cv::Point, 3> warning{{{101, 6}, {106, 2}, {111, 6}}};
    cv::fillConvexPoly(image, warning, 255, cv::LINE_8);
  }
}

void DrawDoorwayActivity(cv::Mat& image, double elapsedSeconds, std::uint32_t seed)
{
  const int eventSecond = static_cast<int>(elapsedSeconds) % DOOR_EVENT_SECONDS;
  if (eventSecond == 16)
  {
    const int step = static_cast<int>(elapsedSeconds * 5.0) % 5;
    cv::rectangle(image, {45, 27 - step * 4, 3, 3}, 255, cv::FILLED, cv::LINE_8);
    cv::rectangle(image, {82, 27 - step * 4, 3, 3}, 255, cv::FILLED, cv::LINE_8);
    cv::rectangle(image, {63, 1, 4, 3}, 255, cv::FILLED, cv::LINE_8);
  }
  else
  {
    const std::uint64_t lampTick = static_cast<std::uint64_t>(elapsedSeconds * 2.0);
    const bool left = ((lampTick + (seed & 1U)) % 2U) == 0U;
    cv::rectangle(image, {left ? 46 : 80, 21, 3, 3}, 255, cv::FILLED, cv::LINE_8);
  }
}
} // namespace

CockpitVisualizer::CockpitVisualizer(CockpitVisualizerConfig config) : config_(config)
{
  const bool finite = std::isfinite(config.target_lit_fraction) &&
                      std::isfinite(config.minimum_lit_fraction) &&
                      std::isfinite(config.maximum_lit_fraction);
  if (config.width != DISPLAY_WIDTH || config.height != DISPLAY_HEIGHT)
    throw std::invalid_argument("cockpit display dimensions must be 128x32");
  if (!finite || config.minimum_lit_fraction <= 0.0 || config.maximum_lit_fraction > 1.0 ||
      config.minimum_lit_fraction > config.target_lit_fraction ||
      config.target_lit_fraction > config.maximum_lit_fraction)
  {
    throw std::invalid_argument("invalid cockpit brightness fractions");
  }

  structure_ = cv::Mat::zeros(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC1);
  DrawCriticalOutlines(structure_);
  const int maximum =
      static_cast<int>(std::floor(config_.maximum_lit_fraction * DISPLAY_WIDTH * DISPLAY_HEIGHT));
  if (cv::countNonZero(structure_) > maximum)
    throw std::logic_error("cockpit doorway structure exceeds configured maximum brightness");
}

cv::Mat CockpitVisualizer::Render(double elapsedSeconds) const
{
  if (!std::isfinite(elapsedSeconds) || elapsedSeconds < 0.0)
    throw std::invalid_argument("elapsed time must be finite and nonnegative");

  cv::Mat frame = structure_.clone();
  DrawLeftConsoleActivity(frame, elapsedSeconds, config_.random_seed);
  DrawRightConsoleActivity(frame, elapsedSeconds, config_.random_seed);
  DrawDoorwayActivity(frame, elapsedSeconds, config_.random_seed);
  cv::bitwise_or(frame, structure_, frame);

  const int pixels = DISPLAY_WIDTH * DISPLAY_HEIGHT;
  const int minimum = static_cast<int>(std::ceil(config_.minimum_lit_fraction * pixels));
  const int maximum = static_cast<int>(std::floor(config_.maximum_lit_fraction * pixels));
  const int lit = cv::countNonZero(frame);
  if (lit < minimum || lit > maximum)
    throw std::runtime_error("configured brightness cannot be represented by cockpit doorway");
  return frame;
}
} // namespace OASIS::Visualization

/*
 *  Copyright (C) 2026 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "cockpit/cockpit_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using OASIS::Visualization::CockpitVisualizer;
using OASIS::Visualization::CockpitVisualizerConfig;

namespace
{
constexpr CockpitVisualizerConfig CONFIG{128, 32, 0.48, 0.40, 0.58, 0x4F415349};

cv::Mat MakeFalconSideViewMask()
{
  cv::Mat mask = cv::Mat::zeros(32, 128, CV_8UC1);
  cv::rectangle(mask, {4, 2, 39, 27}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(mask, {43, 5, 11, 20}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(mask, {76, 5, 9, 20}, 255, cv::FILLED, cv::LINE_8);
  cv::rectangle(mask, {86, 2, 38, 27}, 255, cv::FILLED, cv::LINE_8);

  // Approximate canopy struts and the lower cockpit rim
  cv::line(mask, {31, 2}, {42, 18}, 0, 3, cv::LINE_8);
  cv::line(mask, {87, 17}, {98, 2}, 0, 3, cv::LINE_8);
  cv::rectangle(mask, {43, 24, 42, 8}, 0, cv::FILLED, cv::LINE_8);
  return mask;
}

TEST(CockpitVisualizerTest, ProducesBinaryFramesWithinBrightnessBounds)
{
  const CockpitVisualizer visualizer(CONFIG);
  double total = 0.0;
  int darkest = 4096;
  int brightest = 0;
  for (int index = 0; index <= 300; ++index)
  {
    const cv::Mat frame = visualizer.Render(index * 0.1);
    EXPECT_EQ(frame.cols, 128);
    EXPECT_EQ(frame.rows, 32);
    EXPECT_EQ(frame.type(), CV_8UC1);
    cv::Mat invalid;
    cv::inRange(frame, cv::Scalar(1), cv::Scalar(254), invalid);
    EXPECT_EQ(cv::countNonZero(invalid), 0);
    const int lit = cv::countNonZero(frame);
    EXPECT_GE(lit, std::ceil(0.40 * 4096));
    EXPECT_LE(lit, std::floor(0.58 * 4096));
    darkest = std::min(darkest, lit);
    brightest = std::max(brightest, lit);
    total += lit;
  }
  const double average = total / 301.0;
  RecordProperty("average_lit_pixels", average);
  RecordProperty("minimum_lit_pixels", darkest);
  RecordProperty("maximum_lit_pixels", brightest);
  EXPECT_NEAR(average, 0.48 * 4096, 0.05 * 4096);
}

TEST(CockpitVisualizerTest, IsDeterministicButVariesBySeedAndTime)
{
  const CockpitVisualizer first(CONFIG);
  const CockpitVisualizer second(CONFIG);
  CockpitVisualizerConfig otherConfig = CONFIG;
  otherConfig.random_seed++;
  const CockpitVisualizer other(otherConfig);
  EXPECT_EQ(cv::norm(first.Render(7.25), second.Render(7.25), cv::NORM_INF), 0.0);
  EXPECT_GT(cv::norm(first.Render(7.25), other.Render(7.25), cv::NORM_L1), 0.0);
  EXPECT_GT(cv::norm(first.Render(0.0), first.Render(2.0), cv::NORM_L1), 0.0);
  cv::Mat firstPersistent = first.Render(0.0);
  cv::Mat otherPersistent = other.Render(0.0);
  for (int index = 1; index <= 80; ++index)
  {
    cv::bitwise_and(firstPersistent, first.Render(index * 0.25), firstPersistent);
    cv::bitwise_and(otherPersistent, other.Render(index * 0.25), otherPersistent);
  }
  cv::Mat commonPersistent;
  cv::bitwise_and(firstPersistent, otherPersistent, commonPersistent);
  EXPECT_GE(cv::countNonZero(commonPersistent), static_cast<int>(0.25 * 4096));
  EXPECT_NO_THROW(first.Render(1.0e9));
}

TEST(CockpitVisualizerTest, AnimatesRepeatedlyAcrossConsecutiveTicks)
{
  const CockpitVisualizer visualizer(CONFIG);
  cv::Mat previous = visualizer.Render(0.0);
  std::vector<cv::Mat> uniqueFrames{previous};
  int changes = 0;
  for (int index = 1; index <= 40; ++index)
  {
    const cv::Mat current = visualizer.Render(index * 0.1);
    changes += cv::norm(previous, current, cv::NORM_L1) > 0.0 ? 1 : 0;
    const bool duplicate =
        std::any_of(uniqueFrames.begin(), uniqueFrames.end(), [&current](const cv::Mat& frame)
                    { return cv::norm(frame, current, cv::NORM_INF) == 0.0; });
    if (!duplicate)
      uniqueFrames.push_back(current);
    previous = current;
  }
  RecordProperty("consecutive_frame_changes", changes);
  RecordProperty("unique_frames", uniqueFrames.size());
  EXPECT_GE(changes, 8);
  EXPECT_GE(uniqueFrames.size(), 6U);
}

TEST(CockpitVisualizerTest, PersistentBackgroundDominatesLocalizedAnimation)
{
  const CockpitVisualizer visualizer(CONFIG);
  cv::Mat previous = visualizer.Render(0.0);
  cv::Mat persistent = previous.clone();
  cv::Mat litUnion = previous.clone();
  std::vector<cv::Mat> uniqueFrames{previous};
  double totalLit = cv::countNonZero(previous);
  int changedTransitions = 0;
  int totalChangedPixels = 0;
  int maximumChangedPixels = 0;
  bool hasLocalizedTransition = false;
  for (int index = 1; index <= 50; ++index)
  {
    const cv::Mat current = visualizer.Render(index * 0.1);
    cv::bitwise_and(persistent, current, persistent);
    cv::bitwise_or(litUnion, current, litUnion);
    totalLit += cv::countNonZero(current);

    cv::Mat difference;
    cv::compare(previous, current, difference, cv::CMP_NE);
    const int changed = cv::countNonZero(difference);
    if (changed > 0)
    {
      ++changedTransitions;
      totalChangedPixels += changed;
      maximumChangedPixels = std::max(maximumChangedPixels, changed);
      std::vector<cv::Point> locations;
      cv::findNonZero(difference, locations);
      const cv::Rect changedBounds = cv::boundingRect(locations);
      hasLocalizedTransition |= changedBounds.width < 64 || changedBounds.height < 12;
    }

    const bool duplicate =
        std::any_of(uniqueFrames.begin(), uniqueFrames.end(), [&current](const cv::Mat& frame)
                    { return cv::norm(frame, current, cv::NORM_INF) == 0.0; });
    if (!duplicate)
      uniqueFrames.push_back(current);
    previous = current;
  }

  const int persistentLit = cv::countNonZero(persistent);
  const double averageLit = totalLit / 51.0;
  const double persistentShare = persistentLit / averageLit;
  const double persistentToUnion = static_cast<double>(persistentLit) / cv::countNonZero(litUnion);
  const double averageChanged =
      static_cast<double>(totalChangedPixels) / std::max(changedTransitions, 1);
  RecordProperty("persistent_lit_pixels", persistentLit);
  RecordProperty("persistent_share", persistentShare);
  RecordProperty("persistent_to_union", persistentToUnion);
  RecordProperty("average_changed_pixels", averageChanged);
  RecordProperty("maximum_changed_pixels", maximumChangedPixels);
  RecordProperty("five_second_unique_frames", uniqueFrames.size());
  EXPECT_GE(persistentLit, static_cast<int>(0.25 * 4096));
  EXPECT_GE(persistentShare, 0.5);
  EXPECT_GE(persistentToUnion, 0.55);
  EXPECT_LE(maximumChangedPixels, static_cast<int>(0.20 * 4096));
  EXPECT_TRUE(hasLocalizedTransition);
  EXPECT_GE(uniqueFrames.size(), 8U);
}

TEST(CockpitVisualizerTest, ChangesRemainLimitedAtTenHertz)
{
  const CockpitVisualizer visualizer(CONFIG);
  cv::Mat previous = visualizer.Render(0.0);
  int changes = 0;
  int maximumChanged = 0;
  for (int index = 1; index <= 50; ++index)
  {
    const cv::Mat current = visualizer.Render(index * 0.1);
    cv::Mat difference;
    cv::compare(previous, current, difference, cv::CMP_NE);
    const int changed = cv::countNonZero(difference);
    changes += changed > 0 ? 1 : 0;
    maximumChanged = std::max(maximumChanged, changed);
    previous = current;
  }
  RecordProperty("ten_hz_changes", changes);
  RecordProperty("ten_hz_maximum_changed", maximumChanged);
  EXPECT_GE(changes, 20);
  EXPECT_LT(maximumChanged, static_cast<int>(0.20 * 4096));
  EXPECT_GT(cv::norm(visualizer.Render(0.0), visualizer.Render(1.0), cv::NORM_L1), 0.0);
}

TEST(CockpitVisualizerTest, FalconSideViewPreservesLandmarksAndActivity)
{
  const CockpitVisualizer visualizer(CONFIG);
  const cv::Mat mask = MakeFalconSideViewMask();
  const cv::Mat initial = visualizer.Render(0.0);
  cv::Mat persistent;
  cv::bitwise_and(initial, mask, persistent);
  cv::Mat visibleChanges = cv::Mat::zeros(initial.size(), initial.type());
  for (int index = 1; index <= 50; ++index)
  {
    cv::Mat masked;
    cv::bitwise_and(visualizer.Render(index * 0.1), mask, masked);
    cv::bitwise_and(persistent, masked, persistent);
    cv::Mat difference;
    cv::compare(initial, visualizer.Render(index * 0.1), difference, cv::CMP_NE);
    cv::bitwise_and(difference, mask, difference);
    cv::bitwise_or(visibleChanges, difference, visibleChanges);
  }

  for (const cv::Rect visibleZone : {cv::Rect{4, 2, 39, 27}, cv::Rect{86, 2, 38, 27}})
  {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components =
        cv::connectedComponentsWithStats(persistent(visibleZone), labels, stats, centroids, 8);
    int largest = 0;
    bool hasDistanceScaleFeature = false;
    for (int label = 1; label < components; ++label)
    {
      largest = std::max(largest, stats.at<int>(label, cv::CC_STAT_AREA));
      hasDistanceScaleFeature |= stats.at<int>(label, cv::CC_STAT_WIDTH) >= 10 ||
                                 stats.at<int>(label, cv::CC_STAT_HEIGHT) >= 10;
    }
    EXPECT_GT(largest, 40);
    EXPECT_TRUE(hasDistanceScaleFeature);
  }

  const int maskedPersistent = cv::countNonZero(persistent);
  const int maskedChanges = cv::countNonZero(visibleChanges);
  RecordProperty("side_view_persistent_pixels", maskedPersistent);
  RecordProperty("side_view_changed_pixels", maskedChanges);
  EXPECT_GT(cv::countNonZero(visibleChanges(cv::Rect(4, 2, 39, 27))), 10);
  EXPECT_GT(cv::countNonZero(visibleChanges(cv::Rect(43, 5, 42, 20))), 5);
  EXPECT_GT(cv::countNonZero(visibleChanges(cv::Rect(86, 2, 38, 27))), 10);
}

TEST(CockpitVisualizerTest, PreservesDoorwayAndIndependentConsoleActivity)
{
  const CockpitVisualizer visualizer(CONFIG);
  const cv::Mat initial = visualizer.Render(0.0);
  cv::Mat persistent = initial.clone();
  cv::Mat changes = cv::Mat::zeros(initial.size(), initial.type());
  for (int index = 1; index <= 80; ++index)
  {
    const cv::Mat frame = visualizer.Render(index * 0.125);
    cv::bitwise_and(persistent, frame, persistent);
    cv::Mat difference;
    cv::compare(initial, frame, difference, cv::CMP_NE);
    cv::bitwise_or(changes, difference, changes);
  }

  // Stepped outer arch, inner shoulders, jambs, and threshold
  EXPECT_EQ(persistent.at<std::uint8_t>(3, 64), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(7, 54), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(7, 75), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(20, 44), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(20, 85), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(30, 48), 255);
  EXPECT_EQ(persistent.at<std::uint8_t>(30, 81), 255);

  // The conservative doorway interior stays almost completely black
  int doorwayLit = 0;
  for (int index = 0; index <= 100; ++index)
    doorwayLit = std::max(
        doorwayLit, cv::countNonZero(visualizer.Render(index * 0.2)(cv::Rect(54, 10, 21, 21))));
  RecordProperty("doorway_black_fraction", 1.0 - static_cast<double>(doorwayLit) / (21 * 21));
  EXPECT_LE(doorwayLit, static_cast<int>(0.15 * 21 * 21));

  EXPECT_GT(cv::countNonZero(changes(cv::Rect(2, 2, 39, 28))), 20);
  EXPECT_GT(cv::countNonZero(changes(cv::Rect(43, 1, 43, 31))), 5);
  EXPECT_GT(cv::countNonZero(changes(cv::Rect(87, 1, 38, 29))), 20);

  // The deterministic door activation lights the frame, then returns to normal
  const cv::Mat beforeEvent = visualizer.Render(15.5);
  const cv::Mat event = visualizer.Render(16.4);
  const cv::Mat afterEvent = visualizer.Render(17.5);
  EXPECT_GT(cv::norm(beforeEvent, event, cv::NORM_L1), 0.0);
  EXPECT_GT(cv::norm(event, afterEvent, cv::NORM_L1), 0.0);
  EXPECT_LE(cv::countNonZero(event(cv::Rect(54, 10, 21, 21))), static_cast<int>(0.15 * 21 * 21));
}

TEST(CockpitVisualizerTest, SliderReciprocatesAndRightSequencerMovesQuickly)
{
  const CockpitVisualizer visualizer(CONFIG);
  const auto sliderCenter = [&visualizer](double time)
  {
    const cv::Mat row = visualizer.Render(time)(cv::Rect(8, 13, 27, 1));
    std::vector<cv::Point> locations;
    cv::findNonZero(row, locations);
    EXPECT_FALSE(locations.empty());
    if (locations.empty())
      return -1;
    return cv::boundingRect(locations).x + cv::boundingRect(locations).width / 2;
  };
  const int left = sliderCenter(0.0);
  const int right = sliderCenter(2.625);
  const int returned = sliderCenter(5.25);
  EXPECT_GT(right, left + 10);
  EXPECT_NEAR(returned, left, 1);

  std::vector<cv::Mat> uniqueRightStates;
  cv::Mat previousRight = visualizer.Render(0.0)(cv::Rect(87, 1, 38, 28)).clone();
  uniqueRightStates.push_back(previousRight);
  int rightChanges = 0;
  int obviousChanges = 0;
  int minimumChanged = 4096;
  int maximumChanged = 0;
  int differingColumns = 0;
  for (int index = 1; index <= 20; ++index)
  {
    const cv::Mat frame = visualizer.Render(index * 0.1);
    const cv::Mat right = frame(cv::Rect(87, 1, 38, 28));
    cv::Mat difference;
    cv::compare(previousRight, right, difference, cv::CMP_NE);
    const int changed = cv::countNonZero(difference);
    if (changed > 0)
    {
      ++rightChanges;
      obviousChanges += changed >= 20 ? 1 : 0;
      minimumChanged = std::min(minimumChanged, changed);
      maximumChanged = std::max(maximumChanged, changed);
    }
    const bool duplicate = std::any_of(uniqueRightStates.begin(), uniqueRightStates.end(),
                                       [&right](const cv::Mat& state)
                                       { return cv::norm(state, right, cv::NORM_INF) == 0.0; });
    if (!duplicate)
      uniqueRightStates.push_back(right.clone());
    differingColumns +=
        cv::norm(frame(cv::Rect(93, 9, 9, 18)), frame(cv::Rect(110, 9, 9, 18)), cv::NORM_L1) > 0.0
            ? 1
            : 0;
    previousRight = right.clone();
  }
  RecordProperty("right_two_second_unique_states", uniqueRightStates.size());
  RecordProperty("right_minimum_changed_pixels", minimumChanged);
  RecordProperty("right_maximum_changed_pixels", maximumChanged);
  EXPECT_GE(rightChanges, 8);
  EXPECT_GE(uniqueRightStates.size(), 6U);
  EXPECT_GE(obviousChanges, 5);
  EXPECT_LE(maximumChanged, static_cast<int>(0.20 * 4096));
  EXPECT_GE(differingColumns, 12);

  cv::Mat initialRight = visualizer.Render(0.07)(cv::Rect(87, 1, 38, 28)).clone();
  double firstChangeLatency = 1.0;
  for (int step = 1; step <= 3; ++step)
  {
    const double latency = step * 0.1;
    const cv::Mat right = visualizer.Render(0.07 + latency)(cv::Rect(87, 1, 38, 28));
    if (cv::norm(initialRight, right, cv::NORM_L1) > 0.0)
    {
      firstChangeLatency = latency;
      break;
    }
  }
  RecordProperty("right_first_change_latency", firstChangeLatency);
  EXPECT_LE(firstChangeLatency, 0.3);

  std::vector<int> primaryCounts;
  for (int tick = 0; tick <= 10; ++tick)
    primaryCounts.push_back(
        cv::countNonZero(visualizer.Render(tick / 5.0 + 0.01)(cv::Rect(93, 9, 9, 18))));
  const auto peak = std::max_element(primaryCounts.begin(), primaryCounts.end());
  EXPECT_GT(std::distance(primaryCounts.begin(), peak), 1);
  EXPECT_LT(std::distance(primaryCounts.begin(), peak), 9);
  EXPECT_GT(*peak, primaryCounts.front() + 25);
  EXPECT_NEAR(primaryCounts.back(), primaryCounts.front(), 12);

  const cv::Mat beforeRightUpdate = visualizer.Render(0.59);
  const cv::Mat afterRightUpdate = visualizer.Render(0.61);
  cv::Mat rightOnlyDifference;
  cv::compare(beforeRightUpdate, afterRightUpdate, rightOnlyDifference, cv::CMP_NE);
  EXPECT_EQ(cv::countNonZero(rightOnlyDifference(cv::Rect(2, 2, 39, 28))), 0);
  EXPECT_GT(cv::countNonZero(rightOnlyDifference(cv::Rect(87, 1, 38, 29))), 0);
}

TEST(CockpitVisualizerTest, UsesLargePanelsMultipleScalesAndMeaningfulBlackouts)
{
  const CockpitVisualizer visualizer(CONFIG);
  const cv::Mat frame = visualizer.Render(7.25);

  cv::Mat litLabels;
  cv::Mat litStats;
  cv::Mat litCentroids;
  const int litComponents =
      cv::connectedComponentsWithStats(frame, litLabels, litStats, litCentroids, 8);
  int largestLit = 0;
  for (int label = 1; label < litComponents; ++label)
    largestLit = std::max(largestLit, litStats.at<int>(label, cv::CC_STAT_AREA));
  EXPECT_GT(largestLit, 16);

  cv::Mat darkness;
  cv::compare(frame, 0, darkness, cv::CMP_EQ);
  cv::Mat darkLabels;
  cv::Mat darkStats;
  cv::Mat darkCentroids;
  const int darkComponents =
      cv::connectedComponentsWithStats(darkness, darkLabels, darkStats, darkCentroids, 8);
  bool hasLargeBlackout = false;
  for (int label = 1; label < darkComponents; ++label)
  {
    const int area = darkStats.at<int>(label, cv::CC_STAT_AREA);
    const int width = darkStats.at<int>(label, cv::CC_STAT_WIDTH);
    const int height = darkStats.at<int>(label, cv::CC_STAT_HEIGHT);
    hasLargeBlackout |= area > 48 && width >= 6 && height >= 4;
  }
  EXPECT_TRUE(hasLargeBlackout);

  bool hasSmallRun = false;
  bool hasLargeRun = false;
  for (int y = 2; y < frame.rows - 2; ++y)
  {
    int run = 0;
    for (int x = 2; x < frame.cols - 1; ++x)
    {
      if (frame.at<std::uint8_t>(y, x) != 0)
      {
        ++run;
      }
      else
      {
        hasSmallRun |= run >= 3 && run <= 10;
        hasLargeRun |= run >= 8;
        run = 0;
      }
    }
  }
  EXPECT_TRUE(hasSmallRun);
  EXPECT_TRUE(hasLargeRun);
}

TEST(CockpitVisualizerTest, RejectsInvalidConfigurationAndTime)
{
  EXPECT_THROW(CockpitVisualizer({0, 32, 0.55, 0.45, 0.68, 1}), std::invalid_argument);
  EXPECT_THROW(CockpitVisualizer({127, 32, 0.55, 0.45, 0.68, 1}), std::invalid_argument);
  EXPECT_THROW(CockpitVisualizer({128, 32, 0.4, 0.5, 0.6, 1}), std::invalid_argument);
  EXPECT_THROW(CockpitVisualizer({128, 32, 0.7, 0.5, 0.6, 1}), std::invalid_argument);
  EXPECT_THROW(CockpitVisualizer({128, 32, 0.5, 0.0, 0.6, 1}), std::invalid_argument);
  EXPECT_THROW(CockpitVisualizer({128, 32, std::numeric_limits<double>::quiet_NaN(), 0.4, 0.6, 1}),
               std::invalid_argument);
  const CockpitVisualizer visualizer(CONFIG);
  EXPECT_THROW(visualizer.Render(-1.0), std::invalid_argument);
}
} // namespace

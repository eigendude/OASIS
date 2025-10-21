/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlam.h"

#include "ros/RosUtils.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <MapPoint.h>
#include <System.h>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr int MAP_IMAGE_WIDTH = 640;
constexpr int MAP_IMAGE_HEIGHT = 640;
constexpr int MAP_IMAGE_MARGIN = 40;

cv::Scalar CreateColor(unsigned char b, unsigned char g, unsigned char r)
{
  return cv::Scalar(b, g, r);
}

bool ExtractMapPointPosition(const ORB_SLAM3::MapPoint* mapPoint, Eigen::Vector3f& position)
{
  if (mapPoint == nullptr)
    return false;

  if (const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
    return false;

  position = const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->GetWorldPos();

  return std::isfinite(position.x()) && std::isfinite(position.y()) && std::isfinite(position.z());
}
} // namespace

MonocularSlam::MonocularSlam(rclcpp::Node& node, const std::string& mapTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  if (!mapTopic.empty())
  {
    try
    {
      m_mapPublisher = std::make_unique<image_transport::Publisher>(
          image_transport::create_publisher(&node, mapTopic));
      RCLCPP_INFO(*m_logger, "Publishing SLAM map visualization on topic: %s", mapTopic.c_str());
    }
    catch (const std::exception& err)
    {
      RCLCPP_ERROR(*m_logger, "Failed to create map image publisher '%s': %s", mapTopic.c_str(),
                   err.what());
      m_mapPublisher.reset();
    }
  }
}

MonocularSlam::~MonocularSlam() = default;

bool MonocularSlam::Initialize(const std::string& vocabularyFile, const std::string& settingsFile)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile,
                                               ORB_SLAM3::System::MONOCULAR, false);

  return true;
}

void MonocularSlam::Deinitialize()
{
  if (m_slam)
  {
    // Stop all threads
    m_slam->Shutdown();

    //m_slam->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    //m_slam->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    m_slam.reset();
  }

  m_mapPointPositions.clear();
}

void MonocularSlam::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!m_slam)
    return;

  const std_msgs::msg::Header& header = msg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(cv_ptr->image, timestamp);

  const int trackingState = m_slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = m_slam->GetTrackedMapPoints();
  const std::vector<cv::KeyPoint> trackedKeyPoints = m_slam->GetTrackedKeyPointsUn();

  std::size_t trackedMapPointCount = 0;
  for (const ORB_SLAM3::MapPoint* mapPoint : trackedMapPoints)
  {
    if (mapPoint != nullptr)
      ++trackedMapPointCount;
  }

  const Eigen::Vector3f& translation = cameraPose.translation();
  const Eigen::Quaternionf& quaternion = cameraPose.unit_quaternion();

  // clang-format off
  RCLCPP_INFO(*m_logger,
              "SLAM pose state=%d position=(%.3f, %.3f, %.3f) orientation=(%.4f, %.4f, %.4f, %.4f) tracked=%zu/%zu",
              trackingState,
              translation.x(),
              translation.y(),
              translation.z(),
              quaternion.w(),
              quaternion.x(),
              quaternion.y(),
              quaternion.z(),
              trackedMapPointCount,
              trackedKeyPoints.size());
  // clang-format on

  PublishMapVisualization(header, trackedMapPoints, translation, quaternion);
}

void MonocularSlam::PublishMapVisualization(
    const std_msgs::msg::Header& header,
    const std::vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
    const Eigen::Vector3f& cameraPosition,
    const Eigen::Quaternionf& cameraOrientation)
{
  if (!m_mapPublisher)
    return;

  std::vector<Eigen::Vector3f> trackedPositions;
  trackedPositions.reserve(trackedMapPoints.size());

  for (const ORB_SLAM3::MapPoint* mapPoint : trackedMapPoints)
  {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    if (!ExtractMapPointPosition(mapPoint, position))
      continue;

    trackedPositions.push_back(position);
    m_mapPointPositions[mapPoint] = position;
  }

  for (auto it = m_mapPointPositions.begin(); it != m_mapPointPositions.end();)
  {
    const ORB_SLAM3::MapPoint* mapPoint = it->first;
    if (mapPoint == nullptr || const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
    {
      it = m_mapPointPositions.erase(it);
      continue;
    }

    Eigen::Vector3f& storedPosition = it->second;
    if (!ExtractMapPointPosition(mapPoint, storedPosition))
    {
      it = m_mapPointPositions.erase(it);
      continue;
    }

    ++it;
  }

  if (m_mapPointPositions.empty())
  {
    cv::Mat mapImage(MAP_IMAGE_HEIGHT, MAP_IMAGE_WIDTH, CV_8UC3, CreateColor(16, 16, 16));
    cv::putText(mapImage, "No map points", cv::Point(60, MAP_IMAGE_HEIGHT / 2),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, CreateColor(200, 200, 200), 2, cv::LINE_AA);

    sensor_msgs::msg::Image::SharedPtr mapMsg =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mapImage).toImageMsg();
    m_mapPublisher->publish(mapMsg);
    return;
  }

  float minX = cameraPosition.x();
  float maxX = cameraPosition.x();
  float minZ = cameraPosition.z();
  float maxZ = cameraPosition.z();

  for (const auto& entry : m_mapPointPositions)
  {
    const Eigen::Vector3f& pos = entry.second;
    minX = std::min(minX, pos.x());
    maxX = std::max(maxX, pos.x());
    minZ = std::min(minZ, pos.z());
    maxZ = std::max(maxZ, pos.z());
  }

  constexpr float MIN_RANGE = 1e-2F;
  float rangeX = maxX - minX;
  float rangeZ = maxZ - minZ;
  if (rangeX < MIN_RANGE)
  {
    minX -= 0.5F;
    maxX += 0.5F;
    rangeX = maxX - minX;
  }
  if (rangeZ < MIN_RANGE)
  {
    minZ -= 0.5F;
    maxZ += 0.5F;
    rangeZ = maxZ - minZ;
  }

  cv::Mat mapImage(MAP_IMAGE_HEIGHT, MAP_IMAGE_WIDTH, CV_8UC3, CreateColor(12, 12, 12));

  const auto projectPoint = [&](float x, float z)
  {
    const float normalizedX = (x - minX) / rangeX;
    const float normalizedZ = (z - minZ) / rangeZ;

    const int px =
        static_cast<int>(MAP_IMAGE_MARGIN + normalizedX * (MAP_IMAGE_WIDTH - 2 * MAP_IMAGE_MARGIN));
    const int py = static_cast<int>(
        MAP_IMAGE_HEIGHT -
        (MAP_IMAGE_MARGIN + normalizedZ * (MAP_IMAGE_HEIGHT - 2 * MAP_IMAGE_MARGIN)));

    return cv::Point(std::clamp(px, 0, MAP_IMAGE_WIDTH - 1),
                     std::clamp(py, 0, MAP_IMAGE_HEIGHT - 1));
  };

  // Draw historical map points
  for (const auto& entry : m_mapPointPositions)
  {
    const Eigen::Vector3f& pos = entry.second;
    const cv::Point pixel = projectPoint(pos.x(), pos.z());
    cv::circle(mapImage, pixel, 2, CreateColor(80, 80, 255), cv::FILLED, cv::LINE_AA);
  }

  // Highlight currently tracked points
  for (const Eigen::Vector3f& pos : trackedPositions)
  {
    const cv::Point pixel = projectPoint(pos.x(), pos.z());
    cv::circle(mapImage, pixel, 3, CreateColor(0, 255, 255), cv::FILLED, cv::LINE_AA);
  }

  const cv::Point cameraPixel = projectPoint(cameraPosition.x(), cameraPosition.z());
  cv::circle(mapImage, cameraPixel, 6, CreateColor(0, 0, 255), cv::FILLED, cv::LINE_AA);

  const Eigen::Vector3f forwardVector = cameraOrientation * Eigen::Vector3f::UnitZ();
  const float arrowScale = 0.1F * std::max(rangeX, rangeZ);
  const Eigen::Vector3f arrowEnd3D = cameraPosition + forwardVector * arrowScale;
  const cv::Point arrowEndPixel = projectPoint(arrowEnd3D.x(), arrowEnd3D.z());
  cv::arrowedLine(mapImage, cameraPixel, arrowEndPixel, CreateColor(0, 0, 255), 2, cv::LINE_AA, 0,
                  0.2);

  cv::putText(mapImage, "SLAM map", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
              CreateColor(220, 220, 220), 2, cv::LINE_AA);

  const std::string stats = "Points: " + std::to_string(m_mapPointPositions.size()) +
                            " Tracked: " + std::to_string(trackedPositions.size());
  cv::putText(mapImage, stats, cv::Point(20, MAP_IMAGE_HEIGHT - 20), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              CreateColor(220, 220, 220), 1, cv::LINE_AA);

  auto mapMsg =
      cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, mapImage).toImageMsg();
  m_mapPublisher->publish(mapMsg);
}

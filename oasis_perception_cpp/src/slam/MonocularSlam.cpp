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
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
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
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
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

MonocularSlam::MonocularSlam(rclcpp::Node& node,
                             const std::string& mapTopic,
                             const std::string& debugTopic,
                             const std::string& mapImageTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  if (!mapTopic.empty())
  {
    try
    {
      m_mapPublisher =
          node.create_publisher<sensor_msgs::msg::PointCloud2>(mapTopic, rclcpp::SensorDataQoS());
      RCLCPP_INFO(*m_logger, "Publishing SLAM map point cloud on topic: %s", mapTopic.c_str());
    }
    catch (const std::exception& err)
    {
      RCLCPP_ERROR(*m_logger, "Failed to create map point cloud publisher '%s': %s",
                   mapTopic.c_str(), err.what());
      m_mapPublisher.reset();
    }
  }

  if (!mapImageTopic.empty())
  {
    try
    {
      m_mapImagePublisher = std::make_unique<image_transport::Publisher>(
          image_transport::create_publisher(&node, mapImageTopic));
      RCLCPP_INFO(*m_logger, "Publishing SLAM map image on topic: %s", mapImageTopic.c_str());
    }
    catch (const std::exception& err)
    {
      RCLCPP_ERROR(*m_logger, "Failed to create map image publisher '%s': %s",
                   mapImageTopic.c_str(), err.what());
      m_mapImagePublisher.reset();
    }
  }

  if (!debugTopic.empty())
  {
    try
    {
      m_debugPublisher = std::make_unique<image_transport::Publisher>(
          image_transport::create_publisher(&node, debugTopic));
      RCLCPP_INFO(*m_logger, "Publishing SLAM debug visualization on topic: %s",
                  debugTopic.c_str());
    }
    catch (const std::exception& err)
    {
      RCLCPP_ERROR(*m_logger, "Failed to create debug image publisher '%s': %s", debugTopic.c_str(),
                   err.what());
      m_debugPublisher.reset();
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
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(*m_logger, "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = cv_ptr->image;

  if (rgbImage.type() != CV_8UC3)
  {
    RCLCPP_WARN(*m_logger,
                "Received image converted to unexpected type (type=%d channels=%d). Expected RGB8.",
                rgbImage.type(), rgbImage.channels());
  }

  if (m_debugPublisher && m_debugPublisher->getNumSubscribers() > 0)
  {
    cv::Mat debugImage = rgbImage.clone();

    cv::Mat grayImage;
    cv::cvtColor(rgbImage, grayImage, cv::COLOR_RGB2GRAY);

    std::vector<cv::Point2f> corners;
    constexpr int MAX_CORNERS = 500;
    constexpr double QUALITY_LEVEL = 0.01;
    constexpr double MIN_DISTANCE = 7.0;
    cv::goodFeaturesToTrack(grayImage, corners, MAX_CORNERS, QUALITY_LEVEL, MIN_DISTANCE);

    for (const cv::Point2f& corner : corners)
      cv::circle(debugImage, corner, 3, cv::Scalar(0, 255, 0), cv::FILLED);

    PublishDebugImage(header, debugImage);
  }

  // Pass the image to the SLAM system
  const Sophus::SE3f cameraPose = m_slam->TrackMonocular(rgbImage, timestamp);

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

void MonocularSlam::PublishDebugImage(const std_msgs::msg::Header& header,
                                      const cv::Mat& debugImage)
{
  if (!m_debugPublisher)
    return;

  cv_bridge::CvImage cvImage;
  cvImage.header = header;
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = debugImage;

  m_debugPublisher->publish(cvImage.toImageMsg());
}

void MonocularSlam::PublishMapVisualization(
    const std_msgs::msg::Header& header,
    const std::vector<ORB_SLAM3::MapPoint*>& trackedMapPoints,
    const Eigen::Vector3f& cameraPosition,
    const Eigen::Quaternionf& cameraOrientation)
{
  const bool publishPointCloud = static_cast<bool>(m_mapPublisher);
  const bool publishMapImage = m_mapImagePublisher && m_mapImagePublisher->getNumSubscribers() > 0;

  if (!publishPointCloud && !publishMapImage)
    return;

  std::unordered_set<const ORB_SLAM3::MapPoint*> trackedPointSet;
  trackedPointSet.reserve(trackedMapPoints.size());

  for (const ORB_SLAM3::MapPoint* mapPoint : trackedMapPoints)
  {
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    if (!ExtractMapPointPosition(mapPoint, position))
      continue;

    trackedPointSet.insert(mapPoint);
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

  const bool cameraValid = std::isfinite(cameraPosition.x()) && std::isfinite(cameraPosition.y()) &&
                           std::isfinite(cameraPosition.z());

  std::vector<MapPointRenderInfo> renderPoints;
  renderPoints.reserve(m_mapPointPositions.size());

  std::size_t validMapPointCount = 0;
  float maxDistance = 0.0F;
  for (const auto& entry : m_mapPointPositions)
  {
    const Eigen::Vector3f& position = entry.second;
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z()))
      continue;

    const bool isTracked = trackedPointSet.find(entry.first) != trackedPointSet.end();

    renderPoints.push_back({position, isTracked});

    ++validMapPointCount;

    if (cameraValid)
      maxDistance = std::max(maxDistance, (position - cameraPosition).norm());
  }

  bool arrowValid = false;
  Eigen::Vector3f arrowTip = Eigen::Vector3f::Zero();
  if (cameraValid)
  {
    Eigen::Vector3f forwardVector = cameraOrientation * Eigen::Vector3f::UnitZ();
    const bool forwardFinite = std::isfinite(forwardVector.x()) &&
                               std::isfinite(forwardVector.y()) && std::isfinite(forwardVector.z());
    if (forwardFinite)
    {
      const float arrowScale = std::max(0.25F, 0.1F * maxDistance);
      const float forwardNorm = forwardVector.norm();
      if (forwardNorm > std::numeric_limits<float>::epsilon())
      {
        arrowTip = cameraPosition + forwardVector / forwardNorm * arrowScale;
        if (std::isfinite(arrowTip.x()) && std::isfinite(arrowTip.y()) &&
            std::isfinite(arrowTip.z()))
          arrowValid = true;
      }
    }
  }

  if (publishPointCloud)
  {
    sensor_msgs::msg::PointCloud2 pointCloud;
    pointCloud.header = header;

    std::size_t pointCount = validMapPointCount;
    if (cameraValid)
      ++pointCount;
    if (arrowValid)
      ++pointCount;

    sensor_msgs::PointCloud2Modifier modifier(pointCloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
                                  sensor_msgs::msg::PointField::UINT32);
    modifier.resize(pointCount);

    if (pointCount > 0)
    {
      sensor_msgs::PointCloud2Iterator<float> iterX(pointCloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(pointCloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(pointCloud, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> iterR(pointCloud, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> iterG(pointCloud, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> iterB(pointCloud, "b");

      const auto addPoint =
          [&](const Eigen::Vector3f& position, uint8_t red, uint8_t green, uint8_t blue)
      {
        *iterX = position.x();
        *iterY = position.y();
        *iterZ = position.z();
        *iterR = red;
        *iterG = green;
        *iterB = blue;

        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterR;
        ++iterG;
        ++iterB;
      };

      for (const auto& renderPoint : renderPoints)
      {
        if (renderPoint.tracked)
          addPoint(renderPoint.position, 255, 255, 0);
        else
          addPoint(renderPoint.position, 255, 64, 64);
      }

      if (cameraValid)
        addPoint(cameraPosition, 64, 160, 255);

      if (arrowValid)
        addPoint(arrowTip, 64, 160, 255);
    }

    pointCloud.is_dense = false;

    m_mapPublisher->publish(pointCloud);
  }

  if (publishMapImage)
    PublishMapImage(header, renderPoints, cameraPosition, cameraOrientation, maxDistance);
}

void MonocularSlam::PublishMapImage(const std_msgs::msg::Header& header,
                                    const std::vector<MapPointRenderInfo>& renderPoints,
                                    const Eigen::Vector3f& cameraPosition,
                                    const Eigen::Quaternionf& cameraOrientation,
                                    float maxDistance)
{
  if (!m_mapImagePublisher || m_mapImagePublisher->getNumSubscribers() == 0)
    return;

  if (!cameraPosition.allFinite())
    return;

  if (!cameraOrientation.coeffs().array().isFinite().all())
    return;

  Eigen::Quaternionf orientation = cameraOrientation;
  const float orientationNorm = orientation.norm();
  if (!(orientationNorm > std::numeric_limits<float>::epsilon()))
    return;

  orientation.normalize();

  constexpr int IMAGE_WIDTH = 640;
  constexpr int IMAGE_HEIGHT = 480;
  constexpr float FIELD_OF_VIEW_DEGREES = 90.0F;
  constexpr float PI = 3.14159265358979323846F;
  constexpr float NEAR_PLANE = 0.05F;

  const float fovRadians = FIELD_OF_VIEW_DEGREES * PI / 180.0F;
  const float halfWidth = static_cast<float>(IMAGE_WIDTH) * 0.5F;
  const float halfHeight = static_cast<float>(IMAGE_HEIGHT) * 0.5F;
  const float focalLength = halfWidth / std::tan(fovRadians * 0.5F);
  const float fx = focalLength;
  const float fy = focalLength;
  const float cx = halfWidth;
  const float cy = halfHeight;

  const float farPlane =
      std::max(NEAR_PLANE + 1e-3F, maxDistance > 0.0F ? maxDistance * 1.25F : 5.0F);

  cv::Mat mapImageBgr(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));

  struct ProjectedPoint
  {
    cv::Point pixel;
    float depth = 0.0F;
    float depthFactor = 0.0F;
    bool tracked = false;
  };

  std::vector<ProjectedPoint> projectedPoints;
  projectedPoints.reserve(renderPoints.size());
  std::size_t visibleCount = 0;
  std::size_t trackedVisibleCount = 0;

  for (const auto& point : renderPoints)
  {
    Eigen::Vector3f relative = point.position - cameraPosition;
    if (!relative.allFinite())
      continue;

    Eigen::Vector3f cameraSpace = orientation.conjugate() * relative;
    if (!cameraSpace.allFinite())
      continue;

    if (cameraSpace.z() <= NEAR_PLANE)
      continue;

    const float invZ = 1.0F / cameraSpace.z();
    const float u = fx * cameraSpace.x() * invZ + cx;
    const float v = fy * cameraSpace.y() * invZ + cy;

    if (u < 0.0F || u >= static_cast<float>(IMAGE_WIDTH) || v < 0.0F ||
        v >= static_cast<float>(IMAGE_HEIGHT))
      continue;

    const float clampedDepth = std::min(farPlane, cameraSpace.z());
    float depthFactor = 1.0F - (clampedDepth - NEAR_PLANE) / (farPlane - NEAR_PLANE);
    depthFactor = std::clamp(depthFactor, 0.1F, 1.0F);

    projectedPoints.push_back({
        cv::Point{static_cast<int>(std::round(u)), static_cast<int>(std::round(v))},
        cameraSpace.z(),
        depthFactor,
        point.tracked,
    });

    ++visibleCount;
    if (point.tracked)
      ++trackedVisibleCount;
  }

  std::stable_sort(projectedPoints.begin(), projectedPoints.end(),
                   [](const ProjectedPoint& lhs, const ProjectedPoint& rhs)
                   { return lhs.depth > rhs.depth; });

  for (const ProjectedPoint& point : projectedPoints)
  {
    cv::Scalar colorBgr = point.tracked ? cv::Scalar(0, 255, 255) : cv::Scalar(64, 64, 255);
    colorBgr[0] = std::clamp(static_cast<int>(colorBgr[0] * point.depthFactor), 0, 255);
    colorBgr[1] = std::clamp(static_cast<int>(colorBgr[1] * point.depthFactor), 0, 255);
    colorBgr[2] = std::clamp(static_cast<int>(colorBgr[2] * point.depthFactor), 0, 255);

    const int radius =
        std::clamp(static_cast<int>(std::round(2.0F + 6.0F * point.depthFactor)), 1, 8);

    cv::circle(mapImageBgr, point.pixel, radius, colorBgr, cv::FILLED, cv::LINE_AA);
  }

  cv::line(mapImageBgr, cv::Point(IMAGE_WIDTH / 2, 0), cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT),
           cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  cv::line(mapImageBgr, cv::Point(0, IMAGE_HEIGHT / 2), cv::Point(IMAGE_WIDTH, IMAGE_HEIGHT / 2),
           cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
  cv::circle(mapImageBgr, cv::Point(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2), 6, cv::Scalar(80, 80, 80),
             1, cv::LINE_AA);

  std::string overlayText = "Visible: " + std::to_string(visibleCount) +
                            " | Tracked: " + std::to_string(trackedVisibleCount);
  cv::putText(mapImageBgr, overlayText, cv::Point(8, IMAGE_HEIGHT - 12), cv::FONT_HERSHEY_SIMPLEX,
              0.45, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);

  cv::Mat mapImageRgb;
  cv::cvtColor(mapImageBgr, mapImageRgb, cv::COLOR_BGR2RGB);

  cv_bridge::CvImage cvImage;
  cvImage.header = header;
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = mapImageRgb;

  m_mapImagePublisher->publish(cvImage.toImageMsg());
}

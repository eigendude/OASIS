/*
 *  Copyright (C) 2021-2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "MonocularSlamBase.h"

#include "ros/RosUtils.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace OASIS;
using namespace SLAM;

namespace
{
constexpr char MAP_FRAME_ID[] = "map";
} // namespace

MonocularSlamBase::MonocularSlamBase(rclcpp::Node& node,
                                     const std::string& mapImageTopic,
                                     const std::string& pointCloudTopic,
                                     const std::string& poseTopic)
  : m_logger(std::make_unique<rclcpp::Logger>(node.get_logger()))
{
  rclcpp::QoS sensorQos = rclcpp::SensorDataQoS();
  sensorQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  m_mapImagePublisher =
      image_transport::create_publisher(&node, mapImageTopic, sensorQos.get_rmw_qos_profile());

  rclcpp::QoS pointCloudQos{rclcpp::SensorDataQoS{}};
  pointCloudQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  m_pointCloudPublisher =
      node.create_publisher<sensor_msgs::msg::PointCloud2>(pointCloudTopic, pointCloudQos);

  rclcpp::QoS poseQos{rclcpp::SensorDataQoS{}};
  poseQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  m_posePublisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(poseTopic, poseQos);

  m_renderThreadRunning = true;
  m_renderThread = std::thread(&MonocularSlamBase::MapPublisherLoop, this);
}

MonocularSlamBase::~MonocularSlamBase()
{
  StopMapPublisher();
}

bool MonocularSlamBase::InitializeSystem(const std::string& vocabularyFile,
                                         const std::string& settingsFile,
                                         ORB_SLAM3::System::eSensor sensorType)
{
  if (vocabularyFile.empty() || settingsFile.empty())
    return false;

  if (!LoadCameraModel(settingsFile, m_cameraModel, Logger()))
    return false;

  m_mapViewRenderer.Initialize(m_cameraModel);

  m_slam = std::make_unique<ORB_SLAM3::System>(vocabularyFile, settingsFile, sensorType, false);

  m_lastTimestamp.reset();

  return true;
}

void MonocularSlamBase::DeinitializeSystem()
{
  if (m_slam)
  {
    m_slam->Shutdown();
    m_slam.reset();
  }

  m_lastTimestamp.reset();

  {
    std::lock_guard<std::mutex> lock(m_renderMutex);
    m_renderQueue.clear();
  }
}

void MonocularSlamBase::ReceiveImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (!HasSlam())
    return;

  const std_msgs::msg::Header& header = msg->header;
  const double timestamp = ROS::RosUtils::HeaderStampToSeconds(header);

  if (m_lastTimestamp && timestamp <= *m_lastTimestamp)
  {
    RCLCPP_WARN(Logger(), "Rejecting frame with non-increasing timestamp %.6f (last %.6f)",
                timestamp, *m_lastTimestamp);
    return;
  }

  cv_bridge::CvImageConstPtr inputImage;
  try
  {
    inputImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(Logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat& rgbImage = inputImage->image;

  m_lastTimestamp = timestamp;

  const Eigen::Isometry3f cameraPose = TrackFrame(rgbImage, timestamp);

  ORB_SLAM3::System* slam = GetSlam();
  if (slam == nullptr)
    return;

  if (m_posePublisher)
  {
    geometry_msgs::msg::PoseStamped poseMsg;
    poseMsg.header = header;
    poseMsg.header.frame_id = MAP_FRAME_ID;

    const Eigen::Vector3f& translation = cameraPose.translation();
    poseMsg.pose.position.x = static_cast<double>(translation.x());
    poseMsg.pose.position.y = static_cast<double>(translation.y());
    poseMsg.pose.position.z = static_cast<double>(translation.z());

    Eigen::Quaternionf quaternion(cameraPose.linear());
    quaternion.normalize();
    poseMsg.pose.orientation.x = static_cast<double>(quaternion.x());
    poseMsg.pose.orientation.y = static_cast<double>(quaternion.y());
    poseMsg.pose.orientation.z = static_cast<double>(quaternion.z());
    poseMsg.pose.orientation.w = static_cast<double>(quaternion.w());

    m_posePublisher->publish(poseMsg);
  }

  // Get SLAM state
  const int trackingState = slam->GetTrackingState();
  const std::vector<ORB_SLAM3::MapPoint*> trackedMapPoints = slam->GetTrackedMapPoints();

  // Get map points
  std::vector<ORB_SLAM3::MapPoint*> mapPoints;
  ORB_SLAM3::Atlas* atlas = slam->GetAtlas();
  if (atlas != nullptr)
    mapPoints = atlas->GetAllMapPoints();

  RCLCPP_INFO(Logger(), "Tracking state: %d, tracked points: %zu, map points: %zu", trackingState,
              trackedMapPoints.size(), mapPoints.size());

  if (m_mapImagePublisher && m_renderThreadRunning.load() && !mapPoints.empty())
  {
    MapRenderTask task;
    task.header = header;
    task.cameraPose = cameraPose;
    task.mapPoints = std::move(mapPoints);
    task.inputImage = std::move(inputImage);

    {
      std::lock_guard<std::mutex> lock(m_renderMutex);
      m_renderQueue.clear();
      m_renderQueue.emplace_back(std::move(task));
    }

    m_renderCv.notify_one();
  }

  OnPostTrack();
}

bool MonocularSlamBase::HasSlam() const
{
  return m_slam != nullptr;
}

ORB_SLAM3::System* MonocularSlamBase::GetSlam()
{
  return m_slam.get();
}

rclcpp::Logger& MonocularSlamBase::Logger()
{
  return *m_logger;
}

const rclcpp::Logger& MonocularSlamBase::Logger() const
{
  return *m_logger;
}

void MonocularSlamBase::MapPublisherLoop()
{
  while (true)
  {
    MapRenderTask task;
    {
      std::unique_lock<std::mutex> lock(m_renderMutex);
      m_renderCv.wait(lock,
                      [this] { return !m_renderThreadRunning.load() || !m_renderQueue.empty(); });

      if (!m_renderThreadRunning.load() && m_renderQueue.empty())
        break;

      task = std::move(m_renderQueue.front());
      m_renderQueue.pop_front();
    }

    // Initialize world points
    m_worldPointBuffer.clear();
    m_worldPointBuffer.reserve(task.mapPoints.size());

    // Get world points
    for (const ORB_SLAM3::MapPoint* mapPoint : task.mapPoints)
    {
      // Validate map point
      if (mapPoint == nullptr || const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->isBad())
        continue;

      const Eigen::Vector3f worldPoint = const_cast<ORB_SLAM3::MapPoint*>(mapPoint)->GetWorldPos();

      // Validate world point
      if (!std::isfinite(worldPoint.x()) || !std::isfinite(worldPoint.y()) ||
          !std::isfinite(worldPoint.z()))
      {
        continue;
      }

      // Record world point
      m_worldPointBuffer.push_back(worldPoint);
    }

    if (m_worldPointBuffer.empty())
      continue;

    if (m_pointCloudPublisher && m_pointCloudPublisher->get_subscription_count() > 0)
      PublishPointCloud(task.header, m_worldPointBuffer);

    if (m_mapImagePublisher && m_mapImagePublisher->getNumSubscribers() > 0)
      PublishMapView(task.header, task.cameraPose, m_worldPointBuffer,
                     const_cast<cv::Mat&>(task.inputImage->image));
  }
}

void MonocularSlamBase::StopMapPublisher()
{
  m_renderThreadRunning = false;

  if (m_renderThread.joinable())
  {
    m_renderCv.notify_all();
    m_renderThread.join();
  }

  {
    std::lock_guard<std::mutex> lock(m_renderMutex);
    m_renderQueue.clear();
  }
}

void MonocularSlamBase::PublishPointCloud(const std_msgs::msg::Header& header,
                                          const std::vector<Eigen::Vector3f>& worldPoints)
{
  sensor_msgs::msg::PointCloud2 pointCloud;
  pointCloud.header = header;
  pointCloud.header.frame_id = MAP_FRAME_ID;
  pointCloud.is_bigendian = false;
  pointCloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(pointCloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(worldPoints.size());

  sensor_msgs::PointCloud2Iterator<float> iterX(pointCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(pointCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(pointCloud, "z");

  for (const Eigen::Vector3f& worldPoint : worldPoints)
  {
    *iterX = worldPoint.x();
    *iterY = worldPoint.y();
    *iterZ = worldPoint.z();
    ++iterX;
    ++iterY;
    ++iterZ;
  }

  m_pointCloudPublisher->publish(pointCloud);
}

void MonocularSlamBase::PublishMapView(const std_msgs::msg::Header& header,
                                       const Eigen::Isometry3f& cameraPose,
                                       const std::vector<Eigen::Vector3f>& worldPoints,
                                       cv::Mat& imageBuffer)
{
  if (m_mapViewRenderer.Render(cameraPose, worldPoints, imageBuffer))
  {
    cv_bridge::CvImage output(header, sensor_msgs::image_encodings::RGB8, imageBuffer);
    m_mapImagePublisher->publish(output.toImageMsg());
  }
}

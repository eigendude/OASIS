/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MapVizNode.h"

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include "slam/CameraModel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace OASIS
{
namespace
{
// ROS topics
constexpr std::string_view POSE_TOPIC_SUFFIX = "slam_pose";
constexpr std::string_view POINT_CLOUD_TOPIC_SUFFIX = "slam_point_cloud";
constexpr std::string_view MAP_IMAGE_TOPIC_SUFFIX = "slam_map_image";
constexpr std::string_view CAMERA_INFO_TOPIC_SUFFIX = "camera_info";
constexpr std::string_view IMAGE_TOPIC_SUFFIX = "image_raw";
constexpr std::string_view DETECTIONS_TOPIC_SUFFIX = "apriltags";

// ROS parameters
constexpr std::string_view SYSTEM_ID_PARAMETER = "system_id";
constexpr std::string_view DEFAULT_SYSTEM_ID = "";

constexpr std::string_view SETTINGS_FILE_PARAMETER = "settings_file";
constexpr std::string_view DEFAULT_SETTINGS_FILE = "";

constexpr std::string_view IMAGE_TOPIC_PARAMETER = "image_topic";
constexpr std::string_view IMAGE_TRANSPORT_PARAMETER = "image_transport";
constexpr std::string_view DEFAULT_IMAGE_TRANSPORT = "raw";

constexpr std::string_view BACKGROUND_ALPHA_PARAMETER = "background_alpha";
constexpr double DEFAULT_BACKGROUND_ALPHA = 0.4;

constexpr std::string_view OUTPUT_ENCODING_PARAMETER = "output_encoding";
constexpr std::string_view DEFAULT_OUTPUT_ENCODING = sensor_msgs::image_encodings::BGR8;

// Synchronization settings
constexpr int SYNC_QUEUE_SIZE = 10;
constexpr double MAX_SYNC_INTERVAL_SECONDS = 0.05;
} // namespace

MapVizNode::MapVizNode(rclcpp::Node& node)
  : m_node(node),
    m_logger(node.get_logger()),
    m_mapImagePublisher(std::make_unique<image_transport::Publisher>()),
    m_imageSubscriber(std::make_shared<image_transport::SubscriberFilter>()),
    m_aprilTagVisualizer(m_logger, m_node.get_clock())
{
  m_node.declare_parameter<std::string>(SYSTEM_ID_PARAMETER.data(), DEFAULT_SYSTEM_ID.data());
  m_node.declare_parameter<std::string>(SETTINGS_FILE_PARAMETER.data(),
                                        DEFAULT_SETTINGS_FILE.data());
  m_node.declare_parameter<std::string>(IMAGE_TOPIC_PARAMETER.data(), "");
  m_node.declare_parameter<std::string>(IMAGE_TRANSPORT_PARAMETER.data(),
                                        DEFAULT_IMAGE_TRANSPORT.data());
  m_node.declare_parameter<double>(BACKGROUND_ALPHA_PARAMETER.data(), DEFAULT_BACKGROUND_ALPHA);
  m_node.declare_parameter<std::string>(OUTPUT_ENCODING_PARAMETER.data(),
                                        DEFAULT_OUTPUT_ENCODING.data());
}

MapVizNode::~MapVizNode() = default;

bool MapVizNode::Initialize()
{
  std::string systemId;
  if (!m_node.get_parameter(SYSTEM_ID_PARAMETER.data(), systemId) || systemId.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty system ID parameter '%s'", SYSTEM_ID_PARAMETER.data());
    return false;
  }

  std::string settingsFile;
  if (!m_node.get_parameter(SETTINGS_FILE_PARAMETER.data(), settingsFile) || settingsFile.empty())
  {
    RCLCPP_ERROR(m_logger, "Missing or empty settings file parameter '%s'",
                 SETTINGS_FILE_PARAMETER.data());
    return false;
  }

  if (!SLAM::LoadCameraModel(settingsFile, m_cameraModel, m_logger))
  {
    RCLCPP_ERROR(m_logger, "Failed to load camera model from settings file: %s",
                 settingsFile.c_str());
    return false;
  }

  m_renderer.Initialize(m_cameraModel);

  std::string imageTopic;
  if (!m_node.get_parameter(IMAGE_TOPIC_PARAMETER.data(), imageTopic) || imageTopic.empty())
  {
    imageTopic = systemId;
    imageTopic.push_back('_');
    imageTopic.append(IMAGE_TOPIC_SUFFIX);
  }

  std::string imageTransport;
  if (!m_node.get_parameter(IMAGE_TRANSPORT_PARAMETER.data(), imageTransport) ||
      imageTransport.empty())
  {
    imageTransport = DEFAULT_IMAGE_TRANSPORT;
  }

  double backgroundAlpha = DEFAULT_BACKGROUND_ALPHA;
  m_node.get_parameter(BACKGROUND_ALPHA_PARAMETER.data(), backgroundAlpha);
  m_backgroundAlpha = std::clamp(backgroundAlpha, 0.0, 1.0);

  if (!m_node.get_parameter(OUTPUT_ENCODING_PARAMETER.data(), m_outputEncoding) ||
      m_outputEncoding.empty())
  {
    m_outputEncoding = DEFAULT_OUTPUT_ENCODING;
  }

  std::string poseTopic = systemId;
  poseTopic.push_back('_');
  poseTopic.append(POSE_TOPIC_SUFFIX);

  std::string pointCloudTopic = systemId;
  pointCloudTopic.push_back('_');
  pointCloudTopic.append(POINT_CLOUD_TOPIC_SUFFIX);

  std::string mapImageTopic = systemId;
  mapImageTopic.push_back('_');
  mapImageTopic.append(MAP_IMAGE_TOPIC_SUFFIX);

  std::string cameraInfoTopic = systemId;
  cameraInfoTopic.push_back('_');
  cameraInfoTopic.append(CAMERA_INFO_TOPIC_SUFFIX);

  std::string detectionsTopic = systemId;
  detectionsTopic.push_back('_');
  detectionsTopic.append(DETECTIONS_TOPIC_SUFFIX);

  RCLCPP_INFO(m_logger, "System ID: %s", systemId.c_str());
  RCLCPP_INFO(m_logger, "Camera image topic: %s", imageTopic.c_str());
  RCLCPP_INFO(m_logger, "Image transport: %s", imageTransport.c_str());
  RCLCPP_INFO(m_logger, "Pose topic: %s", poseTopic.c_str());
  RCLCPP_INFO(m_logger, "Point cloud topic: %s", pointCloudTopic.c_str());
  RCLCPP_INFO(m_logger, "Map image topic: %s", mapImageTopic.c_str());
  RCLCPP_INFO(m_logger, "Camera info topic: %s", cameraInfoTopic.c_str());
  RCLCPP_INFO(m_logger, "Detections topic: %s", detectionsTopic.c_str());
  RCLCPP_INFO(m_logger, "Background alpha: %.3f", m_backgroundAlpha);
  RCLCPP_INFO(m_logger, "Output encoding: %s", m_outputEncoding.c_str());
  RCLCPP_INFO(m_logger, "Settings file: %s", settingsFile.c_str());

  *m_mapImagePublisher = image_transport::create_publisher(&m_node, mapImageTopic);

  m_imageSubscriber->subscribe(&m_node, imageTopic, imageTransport,
                               rclcpp::QoS{SYNC_QUEUE_SIZE}.get_rmw_qos_profile());

  m_poseSubscription = m_node.create_subscription<geometry_msgs::msg::PoseStamped>(
      poseTopic, {1},
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) { OnPose(msg); });

  m_pointCloudSubscription =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
          &m_node, pointCloudTopic, rclcpp::QoS{SYNC_QUEUE_SIZE}.get_rmw_qos_profile());

  m_detectionsSubscription =
      std::make_shared<message_filters::Subscriber<apriltag_msgs::msg::AprilTagDetectionArray>>(
          &m_node, detectionsTopic, rclcpp::QoS{SYNC_QUEUE_SIZE}.get_rmw_qos_profile());

  m_imagePointCloudDetectionsSynchronizer =
      std::make_shared<message_filters::Synchronizer<ImagePointCloudDetectionsSyncPolicy>>(
          ImagePointCloudDetectionsSyncPolicy{SYNC_QUEUE_SIZE}, *m_imageSubscriber,
          *m_pointCloudSubscription, *m_detectionsSubscription);
  m_imagePointCloudDetectionsSynchronizer->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(MAX_SYNC_INTERVAL_SECONDS));
  m_imagePointCloudDetectionsSynchronizer->registerCallback(std::bind(
      &MapVizNode::OnImagePointCloudDetections, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));

  m_cameraInfoSubscription = m_node.create_subscription<sensor_msgs::msg::CameraInfo>(
      cameraInfoTopic, {1},
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) { OnCameraInfo(msg); });

  return true;
}

void MapVizNode::Deinitialize()
{
  m_cameraInfoSubscription.reset();
  m_imagePointCloudDetectionsSynchronizer.reset();
  if (m_detectionsSubscription)
  {
    m_detectionsSubscription->unsubscribe();
    m_detectionsSubscription.reset();
  }
  if (m_pointCloudSubscription)
  {
    m_pointCloudSubscription->unsubscribe();
    m_pointCloudSubscription.reset();
  }
  m_poseSubscription.reset();
  if (m_imageSubscriber)
  {
    m_imageSubscriber->unsubscribe();
    m_imageSubscriber.reset();
  }
  m_mapImagePublisher->shutdown();
}

void MapVizNode::OnPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  m_cameraFromWorldTransform = PoseMsgToIsometry(*msg);
}

void MapVizNode::OnImagePointCloudDetections(
    const sensor_msgs::msg::Image::ConstSharedPtr& imageMsg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg,
    const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& detectionsMsg)
{
  if (OnImage(imageMsg, detectionsMsg))
    OnPointCloud(pointCloudMsg);
}

void MapVizNode::OnPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  if (!m_mapImagePublisher || m_mapImagePublisher->getNumSubscribers() == 0)
    return;

  if (!m_cameraFromWorldTransform)
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_node.get_clock(), 5000,
                         "Skipping map viz: waiting for slam pose");
    return;
  }

  std::vector<Eigen::Vector3f> worldPoints;
  if (!PointCloudToVector(*msg, worldPoints) || worldPoints.empty())
    return;

  cv::Mat backgroundImage;
  {
    std::scoped_lock lock(m_backgroundMutex);
    backgroundImage = m_backgroundImage;
  }

  const bool backgroundReady = !backgroundImage.empty() &&
                               backgroundImage.cols == static_cast<int>(m_cameraModel.width) &&
                               backgroundImage.rows == static_cast<int>(m_cameraModel.height);

  if (!m_renderer.Render(*m_cameraFromWorldTransform, worldPoints, m_imageBuffer,
                         backgroundReady ? &backgroundImage : nullptr))
    return;

  const cv::Mat* publishImage = &m_imageBuffer;
  std::string publishEncoding = sensor_msgs::image_encodings::BGR8;

  if (m_outputEncoding == sensor_msgs::image_encodings::RGB8)
  {
    cv::cvtColor(m_imageBuffer, m_outputBuffer, cv::COLOR_BGR2RGB);
    publishImage = &m_outputBuffer;
    publishEncoding = m_outputEncoding;
  }
  else if (m_outputEncoding == sensor_msgs::image_encodings::BGRA8)
  {
    cv::cvtColor(m_imageBuffer, m_outputBuffer, cv::COLOR_BGR2BGRA);
    publishImage = &m_outputBuffer;
    publishEncoding = m_outputEncoding;
  }
  else if (m_outputEncoding == sensor_msgs::image_encodings::RGBA8)
  {
    cv::cvtColor(m_imageBuffer, m_outputBuffer, cv::COLOR_BGR2RGBA);
    publishImage = &m_outputBuffer;
    publishEncoding = m_outputEncoding;
  }
  else if (m_outputEncoding != sensor_msgs::image_encodings::BGR8 && !m_warnedOutputEncoding)
  {
    RCLCPP_WARN(m_logger, "Unsupported output encoding '%s', falling back to BGR8",
                m_outputEncoding.c_str());
    m_warnedOutputEncoding = true;
  }

  cv_bridge::CvImage output(msg->header, publishEncoding, *publishImage);
  m_mapImagePublisher->publish(output.toImageMsg());
}

bool MapVizNode::OnImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                         const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr& detectionsMsg)
{
  if (msg == nullptr)
    return false;

  sensor_msgs::msg::Image::SharedPtr processedImage = m_aprilTagVisualizer.ProcessImage(msg);
  if (processedImage == nullptr)
    return false;

  if (detectionsMsg != nullptr)
  {
    if (sensor_msgs::msg::Image::SharedPtr overlayedImage =
            m_aprilTagVisualizer.ProcessDetections(detectionsMsg))
    {
      processedImage = overlayedImage;
    }
  }

  cv_bridge::CvImageConstPtr cvImage;
  try
  {
    cvImage = cv_bridge::toCvShare(processedImage, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception& exception)
  {
    RCLCPP_ERROR(m_logger, "cv_bridge exception: %s", exception.what());
    return false;
  }

  const cv::Mat& image = cvImage->image;
  if (image.empty())
  {
    RCLCPP_WARN(m_logger, "Received empty background image frame");
    return false;
  }

  cv::Mat darkenedImage;
  image.convertTo(darkenedImage, image.type(), m_backgroundAlpha, 0.0);

  std::scoped_lock lock(m_backgroundMutex);
  m_backgroundImage = darkenedImage;
  m_backgroundHeader = processedImage->header;
  return true;
}

void MapVizNode::OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  if (msg == nullptr)
    return;

  SLAM::CameraModel updatedModel = m_cameraModel;

  if (msg->k.size() >= 6)
  {
    updatedModel.fx = static_cast<float>(msg->k[0]);
    updatedModel.fy = static_cast<float>(msg->k[4]);
    updatedModel.cx = static_cast<float>(msg->k[2]);
    updatedModel.cy = static_cast<float>(msg->k[5]);
  }

  if (msg->width > 0)
    updatedModel.width = msg->width;

  if (msg->height > 0)
    updatedModel.height = msg->height;

  if (updatedModel.width == m_cameraModel.width && updatedModel.height == m_cameraModel.height &&
      updatedModel.fx == m_cameraModel.fx && updatedModel.fy == m_cameraModel.fy &&
      updatedModel.cx == m_cameraModel.cx && updatedModel.cy == m_cameraModel.cy)
  {
    return;
  }

  m_cameraModel = updatedModel;
  m_renderer.Initialize(m_cameraModel);
}

Eigen::Isometry3f MapVizNode::PoseMsgToIsometry(const geometry_msgs::msg::PoseStamped& poseMsg)
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

  const geometry_msgs::msg::Point& position = poseMsg.pose.position;
  transform.translation() =
      Eigen::Vector3f(static_cast<float>(position.x), static_cast<float>(position.y),
                      static_cast<float>(position.z));

  const geometry_msgs::msg::Quaternion& orientation = poseMsg.pose.orientation;
  Eigen::Quaternionf quaternion(
      static_cast<float>(orientation.w), static_cast<float>(orientation.x),
      static_cast<float>(orientation.y), static_cast<float>(orientation.z));
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();

  return transform;
}

bool MapVizNode::PointCloudToVector(const sensor_msgs::msg::PointCloud2& pointCloud,
                                    std::vector<Eigen::Vector3f>& worldPoints)
{
  if (pointCloud.width == 0 || pointCloud.height == 0)
    return false;

  sensor_msgs::PointCloud2ConstIterator<float> iterX(pointCloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iterY(pointCloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iterZ(pointCloud, "z");

  const std::size_t pointCount = static_cast<std::size_t>(pointCloud.width) * pointCloud.height;
  worldPoints.clear();
  worldPoints.reserve(pointCount);

  for (std::size_t index = 0; index < pointCount; ++index, ++iterX, ++iterY, ++iterZ)
  {
    const float x = *iterX;
    const float y = *iterY;
    const float z = *iterZ;

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;

    worldPoints.emplace_back(x, y, z);
  }

  return true;
}

} // namespace OASIS

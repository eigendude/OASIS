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
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
class ViridisPaletteSampler
{
public:
  static Eigen::Vector3f Sample(float t)
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

private:
  struct ViridisSample
  {
    float position = 0.0F;
    Eigen::Vector3f color = Eigen::Vector3f::Zero();
  };

  static const std::array<ViridisSample, 8>& GetSamples()
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
};

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

  const Sophus::SE3f worldPose = cameraPose.inverse();
  const Eigen::Vector3f cameraPosition = worldPose.translation();
  const Eigen::Quaternionf cameraOrientation = worldPose.unit_quaternion();

  // clang-format off
  RCLCPP_INFO(*m_logger,
              "SLAM pose state=%d position=(%.3f, %.3f, %.3f) orientation=(%.4f, %.4f, %.4f, %.4f) tracked=%zu/%zu",
              trackingState,
              cameraPosition.x(),
              cameraPosition.y(),
              cameraPosition.z(),
              cameraOrientation.w(),
              cameraOrientation.x(),
              cameraOrientation.y(),
              cameraOrientation.z(),
              trackedMapPointCount,
              trackedKeyPoints.size());
  // clang-format on

  PublishMapVisualization(header, trackedMapPoints, cameraPosition, cameraOrientation);
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

  for (const auto& entry : m_mapPointPositions)
  {
    const Eigen::Vector3f& position = entry.second;
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
        !std::isfinite(position.z()))
      continue;

    const bool isTracked = trackedPointSet.find(entry.first) != trackedPointSet.end();

    renderPoints.push_back({position, isTracked});
  }

  struct VoxelKey
  {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const VoxelKey&) const = default;
  };

  struct VoxelKeyHasher
  {
    std::size_t operator()(const VoxelKey& key) const noexcept
    {
      constexpr std::size_t PRIME1 = 73856093U;
      constexpr std::size_t PRIME2 = 19349663U;
      constexpr std::size_t PRIME3 = 83492791U;
      return static_cast<std::size_t>(key.x) * PRIME1 ^ static_cast<std::size_t>(key.y) * PRIME2 ^
             static_cast<std::size_t>(key.z) * PRIME3;
    }
  };

  struct VoxelAggregate
  {
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    std::size_t count = 0;
    std::size_t trackedCount = 0;
  };

  std::unordered_map<VoxelKey, VoxelAggregate, VoxelKeyHasher> voxelizedPoints;
  voxelizedPoints.reserve(renderPoints.size());

  constexpr float VOXEL_SIZE_METERS = 0.08F;
  const float invVoxelSize = 1.0F / VOXEL_SIZE_METERS;

  const auto voxelize = [&](const Eigen::Vector3f& position)
  {
    return VoxelKey{static_cast<int>(std::floor(position.x() * invVoxelSize)),
                    static_cast<int>(std::floor(position.y() * invVoxelSize)),
                    static_cast<int>(std::floor(position.z() * invVoxelSize))};
  };

  for (const MapPointRenderInfo& renderPoint : renderPoints)
  {
    const VoxelKey key = voxelize(renderPoint.position);
    VoxelAggregate& aggregate = voxelizedPoints[key];
    aggregate.sum += renderPoint.position;
    ++aggregate.count;
    if (renderPoint.tracked)
      ++aggregate.trackedCount;
  }

  std::vector<MapPointRenderInfo> aggregatedPoints;
  aggregatedPoints.reserve(voxelizedPoints.size());
  for (const auto& entry : voxelizedPoints)
  {
    const VoxelAggregate& aggregate = entry.second;
    if (aggregate.count == 0)
      continue;

    const Eigen::Vector3f averaged = aggregate.sum / static_cast<float>(aggregate.count);
    aggregatedPoints.push_back({averaged, aggregate.trackedCount > 0});
  }

  const std::vector<MapPointRenderInfo>& pointsToRender =
      aggregatedPoints.empty() ? renderPoints : aggregatedPoints;

  std::size_t validMapPointCount = pointsToRender.size();
  float maxDistance = 0.0F;
  float minDistance = std::numeric_limits<float>::max();
  if (cameraValid)
  {
    for (const auto& renderPoint : pointsToRender)
    {
      const float distance = (renderPoint.position - cameraPosition).norm();
      if (!std::isfinite(distance))
        continue;

      maxDistance = std::max(maxDistance, distance);
      minDistance = std::min(minDistance, distance);
    }
    if (!(minDistance < maxDistance))
      minDistance = 0.0F;
  }

  float minHeight = std::numeric_limits<float>::max();
  float maxHeight = std::numeric_limits<float>::lowest();
  for (const auto& renderPoint : pointsToRender)
  {
    minHeight = std::min(minHeight, renderPoint.position.y());
    maxHeight = std::max(maxHeight, renderPoint.position.y());
  }
  if (!(minHeight < maxHeight))
  {
    minHeight = -1.0F;
    maxHeight = 1.0F;
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

  const float heightRange = std::max(maxHeight - minHeight, 1e-3F);

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

      const bool useDepthColors = cameraValid && (minDistance < maxDistance);
      const float distanceRange =
          useDepthColors ? std::max(maxDistance - minDistance, 1e-3F) : 0.0F;

      for (const auto& renderPoint : pointsToRender)
      {
        Eigen::Vector3f color = Eigen::Vector3f::Zero();

        if (useDepthColors)
        {
          const float distance = (renderPoint.position - cameraPosition).norm();
          if (std::isfinite(distance))
          {
            const float depthFactor =
                std::clamp((distance - minDistance) / distanceRange, 0.0F, 1.0F);

            color = ViridisPaletteSampler::Sample(1.0F - depthFactor);
          }
          else
          {
            const float heightFactor =
                std::clamp((renderPoint.position.y() - minHeight) / heightRange, 0.0F, 1.0F);
            color = Eigen::Vector3f((45.0F + heightFactor * 205.0F) / 255.0F,
                                    (80.0F + heightFactor * 140.0F) / 255.0F,
                                    (210.0F - heightFactor * 150.0F) / 255.0F);
          }
        }
        else
        {
          const float heightFactor =
              std::clamp((renderPoint.position.y() - minHeight) / heightRange, 0.0F, 1.0F);
          color = Eigen::Vector3f((45.0F + heightFactor * 205.0F) / 255.0F,
                                  (80.0F + heightFactor * 140.0F) / 255.0F,
                                  (210.0F - heightFactor * 150.0F) / 255.0F);
        }

        const float trackedBoost = renderPoint.tracked ? 1.1F : 0.95F;
        const auto encodeChannel = [&](float base)
        {
          const long value = std::lround(std::clamp(base * trackedBoost, 0.0F, 1.0F) * 255.0F);
          return static_cast<uint8_t>(std::clamp(value, 0L, 255L));
        };

        const uint8_t red = encodeChannel(color.x());
        const uint8_t green = encodeChannel(color.y());
        const uint8_t blue = encodeChannel(color.z());

        addPoint(renderPoint.position, red, green, blue);
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
    PublishMapImage(header, pointsToRender, cameraPosition, cameraOrientation, maxDistance);
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
    float depthNormalized = 0.0F;
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
    const float depthNormalized =
        std::clamp((clampedDepth - NEAR_PLANE) / (farPlane - NEAR_PLANE), 0.0F, 1.0F);
    float depthFactor = 1.0F - depthNormalized;
    depthFactor = std::clamp(depthFactor, 0.1F, 1.0F);

    projectedPoints.push_back({
        cv::Point{static_cast<int>(std::round(u)), static_cast<int>(std::round(v))},
        cameraSpace.z(),
        depthFactor,
        depthNormalized,
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
    Eigen::Vector3f color = ViridisPaletteSampler::Sample(1.0F - point.depthNormalized);
    color *= point.depthFactor;
    color *= point.tracked ? 1.1F : 0.95F;
    color = color.cwiseMax(0.0F).cwiseMin(1.0F);

    const auto toChannel = [](float value)
    {
      const long scaled = std::lround(std::clamp(value, 0.0F, 1.0F) * 255.0F);
      return static_cast<int>(std::clamp(scaled, 0L, 255L));
    };

    cv::Scalar colorBgr(toChannel(color.z()), toChannel(color.y()), toChannel(color.x()));

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

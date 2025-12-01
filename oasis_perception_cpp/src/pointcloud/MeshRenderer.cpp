/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "pointcloud/MeshRenderer.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace OASIS
{
namespace
{
// Image parameters
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;

// Constants
constexpr double PI = 3.14159265358979323846;

// Utility function
cv::Point ProjectPoint(const pcl::PointNormal& point,
                       const pcl::PointNormal& minPoint,
                       const pcl::PointNormal& maxPoint)
{
  const double rangeX = std::max(1e-6f, maxPoint.x - minPoint.x);
  const double rangeY = std::max(1e-6f, maxPoint.y - minPoint.y);

  const double u = (point.x - minPoint.x) / rangeX;
  const double v = (point.y - minPoint.y) / rangeY;

  const int x = static_cast<int>(u * (IMAGE_WIDTH - 1));
  const int y = IMAGE_HEIGHT - 1 - static_cast<int>(v * (IMAGE_HEIGHT - 1));

  return {x, y};
}
} // namespace

MeshRenderer::MeshRenderer(rclcpp::Logger logger,
                           double voxelLeafSize,
                           double normalSearchRadius,
                           double triangulationSearchRadius)
  : m_logger(std::move(logger)),
    m_voxelLeafSize(voxelLeafSize),
    m_normalSearchRadius(normalSearchRadius),
    m_triangulationSearchRadius(triangulationSearchRadius)
{
}

MeshRenderer::~MeshRenderer() = default;

Image::SharedPtr MeshRenderer::Render(const PointCloud2& pointCloud) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pointCloud, *cloud);

  if (cloud->empty())
  {
    RCLCPP_DEBUG(m_logger, "Received empty point cloud");
    return nullptr;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud);
  voxelGrid.setLeafSize(static_cast<float>(m_voxelLeafSize), static_cast<float>(m_voxelLeafSize),
                        static_cast<float>(m_voxelLeafSize));
  voxelGrid.filter(*filtered);

  if (filtered->size() < 3)
  {
    RCLCPP_DEBUG(m_logger, "Downsampled cloud too small for meshing");
    return nullptr;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normalEstimator;
  auto searchTree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  normalEstimator.setInputCloud(filtered);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setRadiusSearch(m_normalSearchRadius);
  normalEstimator.compute(*cloudWithNormals);

  if (cloudWithNormals->size() != filtered->size())
  {
    RCLCPP_WARN(m_logger, "Normal estimation returned unexpected result size");
    return nullptr;
  }

  auto normalsTree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
  normalsTree->setInputCloud(cloudWithNormals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setSearchRadius(m_triangulationSearchRadius);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(PI / 4.0);
  gp3.setMinimumAngle(PI / 18.0);
  gp3.setMaximumAngle(2.0 * PI / 3.0);
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(cloudWithNormals);
  gp3.setSearchMethod(normalsTree);

  pcl::PolygonMesh mesh;
  gp3.reconstruct(mesh);

  if (mesh.polygons.empty())
  {
    RCLCPP_DEBUG(m_logger, "Mesh reconstruction produced no polygons");
    return nullptr;
  }

  pcl::PointCloud<pcl::PointNormal> meshVertices;
  pcl::fromPCLPointCloud2(mesh.cloud, meshVertices);

  if (meshVertices.empty())
  {
    RCLCPP_DEBUG(m_logger, "Mesh reconstruction produced no vertices");
    return nullptr;
  }

  pcl::PointNormal minPoint;
  pcl::PointNormal maxPoint;
  minPoint.x = minPoint.y = std::numeric_limits<float>::max();
  minPoint.z = std::numeric_limits<float>::max();
  maxPoint.x = maxPoint.y = maxPoint.z = std::numeric_limits<float>::lowest();

  for (const pcl::PointNormal& vertex : meshVertices)
  {
    minPoint.x = std::min(minPoint.x, vertex.x);
    minPoint.y = std::min(minPoint.y, vertex.y);
    minPoint.z = std::min(minPoint.z, vertex.z);

    maxPoint.x = std::max(maxPoint.x, vertex.x);
    maxPoint.y = std::max(maxPoint.y, vertex.y);
    maxPoint.z = std::max(maxPoint.z, vertex.z);
  }

  const cv::Scalar backgroundColorBGR(255, 255, 255);
  const cv::Scalar edgeColorBGR(0, 0, 0);
  cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, backgroundColorBGR);

  for (const pcl::Vertices& polygon : mesh.polygons)
  {
    if (polygon.vertices.size() < 3)
      continue;

    const int index0 = polygon.vertices[0];
    const int index1 = polygon.vertices[1];
    const int index2 = polygon.vertices[2];

    if (index0 >= static_cast<int>(meshVertices.size()) ||
        index1 >= static_cast<int>(meshVertices.size()) ||
        index2 >= static_cast<int>(meshVertices.size()))
    {
      continue;
    }

    const cv::Point p0 = ProjectPoint(meshVertices[index0], minPoint, maxPoint);
    const cv::Point p1 = ProjectPoint(meshVertices[index1], minPoint, maxPoint);
    const cv::Point p2 = ProjectPoint(meshVertices[index2], minPoint, maxPoint);

    cv::line(image, p0, p1, edgeColorBGR, 1, cv::LINE_AA);
    cv::line(image, p1, p2, edgeColorBGR, 1, cv::LINE_AA);
    cv::line(image, p2, p0, edgeColorBGR, 1, cv::LINE_AA);
  }

  auto bridge = std::make_shared<cv_bridge::CvImage>(pointCloud.header,
                                                     sensor_msgs::image_encodings::BGR8, image);
  return bridge->toImageMsg();
}
} // namespace OASIS

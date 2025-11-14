/*
 *  Copyright (C) 2025 Garrett Brown
 *  This file is part of OASIS - https://github.com/eigendude/OASIS
 *
 *  SPDX-License-Identifier: Apache-2.0
 *  See the file LICENSE.txt for more information.
 */

#include "nodes/MeshViewerNode.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;

namespace oasis::perception
{
namespace
{
constexpr const char* kInputTopic = "/oasis/falcon/slam_point_cloud";
constexpr const char* kOutputTopic = "/oasis/falcon/slam_mesh_image";

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr double kPi = 3.14159265358979323846;

cv::Point ProjectPoint(const pcl::PointNormal& point,
                       const pcl::PointNormal& min_point,
                       const pcl::PointNormal& max_point)
{
  const double range_x = std::max(1e-6f, max_point.x - min_point.x);
  const double range_y = std::max(1e-6f, max_point.y - min_point.y);

  const double u = (point.x - min_point.x) / range_x;
  const double v = (point.y - min_point.y) / range_y;

  const int x = static_cast<int>(u * (kImageWidth - 1));
  const int y = kImageHeight - 1 - static_cast<int>(v * (kImageHeight - 1));

  return {x, y};
}
} // namespace

MeshViewerNode::MeshViewerNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mesh_viewer_node", options)
{
  mesh_image_publisher_ = create_publisher<Image>(kOutputTopic, rclcpp::SensorDataQoS());

  point_cloud_subscription_ = create_subscription<PointCloud2>(
    kInputTopic,
    rclcpp::SensorDataQoS(),
    std::bind(&MeshViewerNode::OnPointCloud, this, std::placeholders::_1));
}

void MeshViewerNode::OnPointCloud(const PointCloud2::ConstSharedPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty())
  {
    RCLCPP_DEBUG(get_logger(), "Received empty point cloud");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(static_cast<float>(voxel_leaf_size_),
                         static_cast<float>(voxel_leaf_size_),
                         static_cast<float>(voxel_leaf_size_));
  voxel_grid.filter(*filtered);

  if (filtered->size() < 3)
  {
    RCLCPP_DEBUG(get_logger(), "Downsampled cloud too small for meshing");
    return;
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
  auto search_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  normal_estimator.setInputCloud(filtered);
  normal_estimator.setSearchMethod(search_tree);
  normal_estimator.setRadiusSearch(normal_search_radius_);
  normal_estimator.compute(*cloud_with_normals);

  if (cloud_with_normals->size() != filtered->size())
  {
    RCLCPP_WARN(get_logger(), "Normal estimation returned unexpected result size");
    return;
  }

  auto normals_tree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
  normals_tree->setInputCloud(cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setSearchRadius(triangulation_search_radius_);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(kPi / 4.0);
  gp3.setMinimumAngle(kPi / 18.0);
  gp3.setMaximumAngle(2.0 * kPi / 3.0);
  gp3.setNormalConsistency(false);
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(normals_tree);

  pcl::PolygonMesh mesh;
  gp3.reconstruct(mesh);

  if (mesh.polygons.empty())
  {
    RCLCPP_DEBUG(get_logger(), "Mesh reconstruction produced no polygons");
    return;
  }

  pcl::PointCloud<pcl::PointNormal> mesh_vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_vertices);

  if (mesh_vertices.empty())
  {
    RCLCPP_DEBUG(get_logger(), "Mesh reconstruction produced no vertices");
    return;
  }

  pcl::PointNormal min_point;
  pcl::PointNormal max_point;
  min_point.x = min_point.y = std::numeric_limits<float>::max();
  min_point.z = std::numeric_limits<float>::max();
  max_point.x = max_point.y = max_point.z = std::numeric_limits<float>::lowest();

  for (const auto& vertex : mesh_vertices)
  {
    min_point.x = std::min(min_point.x, vertex.x);
    min_point.y = std::min(min_point.y, vertex.y);
    min_point.z = std::min(min_point.z, vertex.z);

    max_point.x = std::max(max_point.x, vertex.x);
    max_point.y = std::max(max_point.y, vertex.y);
    max_point.z = std::max(max_point.z, vertex.z);
  }

  cv::Mat image(kImageHeight, kImageWidth, CV_8UC3, cv::Scalar(255, 255, 255));

  for (const auto& polygon : mesh.polygons)
  {
    if (polygon.vertices.size() < 3)
    {
      continue;
    }

    const auto index0 = polygon.vertices[0];
    const auto index1 = polygon.vertices[1];
    const auto index2 = polygon.vertices[2];

    if (index0 >= mesh_vertices.size() || index1 >= mesh_vertices.size() || index2 >= mesh_vertices.size())
    {
      continue;
    }

    const cv::Point p0 = ProjectPoint(mesh_vertices[index0], min_point, max_point);
    const cv::Point p1 = ProjectPoint(mesh_vertices[index1], min_point, max_point);
    const cv::Point p2 = ProjectPoint(mesh_vertices[index2], min_point, max_point);

    cv::line(image, p0, p1, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::line(image, p1, p2, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    cv::line(image, p2, p0, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  auto bridge = std::make_shared<cv_bridge::CvImage>(msg->header, "bgr8", image);
  auto image_msg = bridge->toImageMsg();

  mesh_image_publisher_->publish(*image_msg);
}
} // namespace oasis::perception

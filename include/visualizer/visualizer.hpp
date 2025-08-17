#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "open3d/Open3D.h"

inline void visualize(const std::vector<Eigen::Vector3d> &pointcloud,
                      Eigen::Matrix4d pose) {
  Eigen::Vector3d origin = Eigen::Vector3d(pose(0, 3), pose(1, 3), pose(2, 3));
  auto mesh_frame =
      open3d::geometry::TriangleMesh::CreateCoordinateFrame(10, origin);
  Eigen::Vector3d origin_0 = Eigen::Vector3d(0, 0, 0);
  auto mesh_frame_0 =
      open3d::geometry::TriangleMesh::CreateCoordinateFrame(5, origin_0);
  std::cout << "origin :\n" << origin << std::endl;
  std::cout << "center :\n" << mesh_frame->GetCenter() << std::endl;
  open3d::visualization::DrawGeometries(
      {std::make_shared<open3d::geometry::PointCloud>(pointcloud), mesh_frame,
       mesh_frame_0},
      "Occupancy Grid Map", 1920, 1080);
}
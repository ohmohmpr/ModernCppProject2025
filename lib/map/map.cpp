#include "map.hpp"
#include "bresenham.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// namespace fs = std::filesystem;

namespace {} // namespace

namespace voxel_map {
Map::Map(const float size_, const float res_) : size(size_), res(res_) {
  shape = static_cast<int>(round(size_ / res_));
  origin_ = {shape / 2, shape / 2, shape / 2};
}

Map::Map(const float size_, const float res_, const float prob_occ_,
         const float prob_free_, const float prior_)
    : size(size_), res(res_), prob_occ(prob_occ_), prob_free(prob_free_),
      prior(prior_) {
  shape = static_cast<int>(round(size_ / res_));
  origin_ = {shape / 2, shape / 2, shape / 2};
}

void Map::Voxelization(Eigen::Matrix4d &pose, const Vector3dVector &cloud) {

  Eigen::Vector3i pose_map_ = pose_to_voxel(pose);
  auto it = voxels_.find(pose_map_);

  // found
  if (it != voxels_.end()) {
    return;
  } else {
    std::cout << "not found" << std::endl;
    cloud_to_map(pose, cloud);
  }
}

void Map::Voxelization(Eigen::Matrix4d &pose) {

  Voxel vox;
  Eigen::Vector3i pose_map_ = pose_to_voxel(pose);

  vox.index_ = pose_map_;
  vox.point_ = pose.col(3).head(3);

  auto it = voxels_.find(pose_map_);

  if (it == voxels_.end()) // not found
    voxels_.insert({pose_map_, vox});
}

std::vector<Eigen::Vector3d> &Map::GetPointCloud() {

  for (auto v : voxels_) {
    points_.emplace_back(v.second.point_);
  };

  return points_;
}

void Map::cloud_to_map(Eigen::Matrix4d &pose, const Vector3dVector &cloud) {

  Eigen::Matrix3d R = pose.block(0, 0, 3, 3);
  Eigen::Vector3d t = pose.col(3).head(3);

  for (auto p : cloud) {
    Eigen::Vector3d pointcloud = R * p + t;
    Eigen::Vector3i pointcloud_index = Eigen::Vector3i();
    pointcloud_index = world_to_map(pointcloud_index, pointcloud);

    Voxel vox;
    vox.index_ = pointcloud_index;
    vox.point_ = pointcloud;

    auto it = voxels_.find(pointcloud_index);

    if (it == voxels_.end()) // not found
      //    // bresenham::Bresenham bresenham =
      //     bresenham::Bresenham(pose_on_map, clouds_on_map[j]);
      // auto clouds_on_mapj = occ_gridmap.Voxelization(clouds_on_map[j]);
      // }
      voxels_.insert({pointcloud_index, vox});
  }
};

Eigen::Vector3i Map::world_to_map(Eigen::Vector3i &pointcloud_index,
                                  Eigen::Vector3d &pointcloud) {

  pointcloud_index.x() =
      pose_map_.x() + static_cast<int>(round(pointcloud(0) / res));
  pointcloud_index.y() =
      pose_map_.y() + static_cast<int>(round(pointcloud(1) / res));
  pointcloud_index.z() =
      pose_map_.z() + static_cast<int>(round(pointcloud(2) / res));

  return pointcloud_index;
};

Eigen::Vector3i Map::world_to_map(Eigen::Matrix4d &pose) {

  pose_map_.x() = origin_.x() + static_cast<int>(round(pose(0, 3) / res));
  pose_map_.y() = origin_.y() + static_cast<int>(round(pose(1, 3) / res));
  pose_map_.z() = origin_.z() + static_cast<int>(round(pose(2, 3) / res));

  return pose_map_;
};

Eigen::Vector3i Map::pose_to_voxel(Eigen::Matrix4d &pose) {
  return world_to_map(pose);
};

// util
void inversion_model(){};
void log_odd(){};

} // namespace voxel_map
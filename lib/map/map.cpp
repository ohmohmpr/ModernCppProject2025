#include "map.hpp"

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

// struct CompareEigenPoints {
//   bool operator()(const Eigen::Vector3i &a, const Eigen::Vector3i &b) const {
//     return std::lexicographical_compare(a.data(), a.data() + 3, b.data(),
//                                         b.data() + 3);
//   }
// };

namespace {} // namespace

namespace voxel_map {
Map::Map(const float size_, const float res_) : size(size_), res(res_) {
  shape = static_cast<int>(round(size_ / res_));
  origin_ = {shape / 2, shape / 2, shape / 2};
}

std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                    std::hash<Eigen::Vector3i>>
Map::Voxelization(const Eigen::Vector3i &indice_on_map) {

  voxels_[indice_on_map] = indice_on_map;
  return voxels_;
}

std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                    std::hash<Eigen::Vector3i>>
Map::Voxelization(Eigen::Matrix4d &pose) {

  Voxel vox;
  Eigen::Vector3i pose_map_ = pose_to_voxel(pose);

  vox.index_ = pose_map_;
  vox.point_ = pose.col(3).head(3);

  voxels_.insert({pose_map_, vox});

  // voxels_[pose_map_] = pose_map_;
  // voxels_[pose_map_].point_ = pose.col(3).head(3);

  return voxels_;
}

std::vector<Eigen::Vector3d> &Map::GetPointCloud() {

  for (auto v : voxels_) {
    points_.emplace_back(v.second.point_);
  };

  return points_;
}

Eigen::Vector3i Map::world_to_map(Eigen::Matrix4d &pose) {

  pose_map_.x() = origin_.x() + static_cast<int>(round(pose(0, 3) / res));
  pose_map_.y() = origin_.y() + static_cast<int>(round(pose(1, 3) / res));
  pose_map_.z() = origin_.z() + static_cast<int>(round(pose(2, 3) / res));

  return pose_map_;
};

Eigen::Vector3i Map::pose_to_voxel(Eigen::Matrix4d &pose) {
  return world_to_map(pose);
};

} // namespace voxel_map
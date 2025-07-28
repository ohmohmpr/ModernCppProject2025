#pragma once

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;

namespace voxel_map {
class Map {
public:
  Map(const float size_, const float res_);
  // std::size_t size() const { return voxels_.size(); }
  int shape;
  Eigen::Vector3i origin() const { return origin_; };
  // PoseAndCloud operator[](const int idx) const;

private:
  std::vector<std::string> pointcloud_files_;
  Vector3dVector voxels_;
  float size = 0;
  float res = 0;
  Eigen::Vector3i origin_;
  // std::unordered_map<Eigen ::Vector3i, Voxel, hash_eigen ::hash<Eigen
  // ::Vector3i>> 4 voxels_;

  // std::unordered_map<Eigen ::Vector3i, hash_eigen::hash<Eigen::Vector3i>>
  // voxels_;
  // how to convert to std::vector<> for visualization
};
} // namespace voxel_map

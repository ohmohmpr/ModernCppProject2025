#pragma once

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;

using std::hash;
template <> struct std::hash<Eigen::Vector3i> {
  std ::size_t operator()(const Eigen::Vector3i &point) const {
    const uint32_t *vec = reinterpret_cast<const uint32_t *>(point.data());
    return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
  }
};

namespace voxel_map {
class Voxel {
public:
  Voxel() {}

  Voxel(const Eigen::Vector3i &index) : index_(index) {}
  ~Voxel() {}

public:
  Eigen::Vector3i index_ = Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3d point_ = Eigen::Vector3d(0, 0, 0);
};

class Map {
public:
  Map(const float size_, const float res_);
  // std::size_t size() const { return voxels_.size(); }
  int shape;
  Eigen::Vector3i origin() const { return origin_; };
  float GetMapRes() { return res; }

  std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                      std::hash<Eigen::Vector3i>>
      voxels_;

  std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                      std::hash<Eigen::Vector3i>>
  Voxelization(const Eigen::Vector3i &indice_on_map);

  std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                      std::hash<Eigen::Vector3i>>
  Voxelization(Eigen::Matrix4d &pose);
  // PoseAndCloud operator[](const int idx) const;

  std::vector<Eigen::Vector3d> &GetPointCloud();
  Eigen::Vector3i world_to_map(Eigen::Matrix4d &pose);
  Eigen::Vector3i pose_to_voxel(Eigen::Matrix4d &pose);

private:
  std::vector<std::string> pointcloud_files_;
  std::vector<Eigen::Vector3d> points_;
  float res = 0;
  float size = 0;
  Eigen::Vector3i origin_;

  // how to convert to std::vector<> for visualization
};
} // namespace voxel_map

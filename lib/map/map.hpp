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
  Map(const float size_, const float res_, const float prob_occ_,
      const float prob_free_, const float prior_);
  // std::size_t size() const { return voxels_.size(); }
  int shape;
  Eigen::Vector3i origin() const { return origin_; };

  std ::unordered_map<Eigen ::Vector3i, voxel_map::Voxel,
                      std::hash<Eigen::Vector3i>>
      voxels_;

  void Voxelization(Eigen::Matrix4d &pose, const Vector3dVector &cloud);

  void Voxelization(Eigen::Matrix4d &pose);
  // PoseAndCloud operator[](const int idx) const;

  std::vector<Eigen::Vector3d> &GetPointCloud();
  Eigen::Vector3i world_to_map(Eigen::Matrix4d &pose);
  Eigen::Vector3i pose_to_voxel(Eigen::Matrix4d &pose);
  Eigen::Vector3i world_to_map(Eigen::Vector3i &pointcloud_index,
                               Eigen::Vector3d &pointcloud);
  void cloud_to_map(Eigen::Matrix4d &pose, const Vector3dVector &cloud);

  void inversion_model(){};
  void log_odd(){};

private:
  std::vector<std::string> pointcloud_files_;
  std::vector<Eigen::Vector3d> points_;
  float res = 0;
  float size = 0;
  Eigen::Vector3i origin_;
  float prob_occ = 0.90;
  float prob_free = 0.35;
  float prior = 0.50;
  Eigen::Vector3i pose_map_ = Eigen::Vector3i();
};
} // namespace voxel_map

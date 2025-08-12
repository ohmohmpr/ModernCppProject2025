#pragma once

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;

namespace bresenham {
class Bresenham {
public:
  Bresenham(Eigen::Vector3i &pose_on_map, Eigen::Vector3i &cloud_on_map);
  // std::size_t size() const { return pointcloud_files_.size(); }
  // PoseAndCloud operator[](const int idx) const;
  std::vector<Eigen::Vector3i> coordinates_int;

private:
  // std::vector<std::string> pointcloud_files_;
};
} // namespace bresenham

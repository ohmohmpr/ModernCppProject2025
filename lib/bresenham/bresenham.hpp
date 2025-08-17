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
  std::vector<Eigen::Vector3i> coordinates_int;

  void find_coordinates(int main_start, int second_axis_start,
                        int third_axis_start, int main_end, int second_axis_end,
                        int third_axis_end,
                        std::vector<Eigen::Vector3i> &coordinates_int);

private:
  int diving_axis_ = -1;
};
} // namespace bresenham

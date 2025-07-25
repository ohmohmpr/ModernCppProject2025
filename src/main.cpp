#include "dataloader.hpp"
#include "open3d/Open3D.h"
#include "visualizer.hpp"
#include <iostream>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
using Dataset = dataloader::Dataset;

int main() {
  const dataloader::Dataset dataset = Dataset("Data");
  const PoseAndCloud pose_and_cloud = dataset[100];
  std::cout << "dataset.size: " << dataset.size() << std::endl;

  std::cout << pose_and_cloud.first << std::endl;
  // visualize(pose_and_cloud.second);
  return 0;
}
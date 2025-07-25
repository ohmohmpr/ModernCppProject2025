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

  // 1. init map here
  // pcc_gridmap()
      
  // pose_and_cloud.first = poses_raw
  // pose_and_cloud.second = ranges_raw
  // occ_gridmap
  // map_res
  // prob_occ
  // prob_free
  // prior
  // 2. grid_mapping_with_known_poses(poses_raw, ranges_raw, occ_gridmap,
  // map_res, prob_occ, prob_free, prior)
  for (int i = 0; i < dataset.size(); i++) {
    std::cout << pose_and_cloud.first << std::endl;
    // grid_mapping_with_known_poses()
  }
  
  // 3. visualize(occ_gridmap);
  return 0;
}
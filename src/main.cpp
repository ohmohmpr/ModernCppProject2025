#include "dataloader.hpp"
#include "map.hpp"
#include "open3d/Open3D.h"
#include "scopedTimer.hpp"
#include "visualizer.hpp"
#include <iostream>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
using Dataset = dataloader::Dataset;

int main() {

  voxel_map::ScopedTimer timer;
  const dataloader::Dataset dataset = Dataset("Data");

  // 1. Init map here
  float map_res = 0.5;
  float prob_occ = 0.90;
  float prob_free = 0.35;
  float prior = 0.50;
  voxel_map::G_Map occ_gridmap =
      voxel_map::G_Map(100, map_res, prob_occ, prob_free, prior);

  // 2. Occupancy Grid mapping
  Eigen::Matrix4d pose_tmp = Eigen::Matrix4d();
  int idx = 4500;
  for (int i = idx; i < idx + 100; i++) {
    // for (int i = 0; i < dataset.size(); i++) {
    std::cout << "frame id: " << i << std::endl;

    const PoseAndCloud pose_and_cloud = dataset[i];
    Eigen::Matrix4d pose = pose_and_cloud.first;
    pose_tmp = pose;
    Vector3dVector cloud = pose_and_cloud.second;

    occ_gridmap.Voxelization(pose, cloud);
    occ_gridmap.Voxelization(pose);
  }

  // 3. Visualize where point cloud with prob > 0.9 is shown.
  std::vector<Eigen::Vector3d> pointcloud = occ_gridmap.GetPointCloud();
  visualize(pointcloud);
  return 0;
}
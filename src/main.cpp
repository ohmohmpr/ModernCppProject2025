#include "bresenham.hpp"
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

  std::cout << "dataset.size: " << dataset.size() << std::endl;

  // 1. init map here
  // occ_gridmap()
  float map_res = 0.5;
  voxel_map::Map occ_gridmap = voxel_map::Map(100, map_res);
  std::cout << occ_gridmap.shape << std::endl;

  // pose_and_cloud.first = poses_raw
  // pose_and_cloud.second = ranges_raw
  // occ_gridmap
  // prob_occ
  // prob_free
  // prior
  // 2. grid_mapping_with_known_poses(poses_raw, ranges_raw, occ_gridmap,
  // map_res, prob_occ, prob_free, prior)
  // for (int i = 0; i < 400; i++) {
  for (int i = 0; i < dataset.size(); i++) {
    std::cout << "frame id: " << i << std::endl;

    const PoseAndCloud pose_and_cloud = dataset[i];
    Eigen::Matrix4d pose = pose_and_cloud.first;
    Vector3dVector cloud = pose_and_cloud.second;

    occ_gridmap.Voxelization(pose, cloud);
    occ_gridmap.Voxelization(pose);

    // std::vector<Eigen::Vector3i> clouds_on_map =
    //     cloud_to_map(cloud, pose, occ_gridmap, map_res);

    // for (int j = 0; j < 1; j++) {
    // for (int j = 0; j < clouds_on_map.size(); j++) {
    //   std::cout << "j: " << j << std::endl;
    //   std::cout << "clouds_on_map.size()j: " << clouds_on_map.size()
    //             << std::endl;

    // bresenham::Bresenham bresenham =
    //     bresenham::Bresenham(pose_on_map, clouds_on_map[j]);
    // auto clouds_on_mapj = occ_gridmap.Voxelization(clouds_on_map[j]);
    // }

    // std::cout << "clouds_on_mapj : " << occ_gridmap.voxels_.size() <<
    // std::endl;

    // grid_mapping_with_known_poses()
  }

  // 3. visualize(occ_gridmap);
  std::vector<Eigen::Vector3d> pointcloud = occ_gridmap.GetPointCloud();
  visualize(pointcloud);
  return 0;
}
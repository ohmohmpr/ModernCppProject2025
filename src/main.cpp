#include "dataloader.hpp"
#include "map.hpp"
#include "bresenham.hpp"
#include "open3d/Open3D.h"
#include "visualizer.hpp"
#include <iostream>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
using Dataset = dataloader::Dataset;

Eigen::Vector3i world_to_map(Eigen::Vector3d &pose, voxel_map::Map &occ_gridmap,
                             float map_res) {

  Eigen::Vector3i pose_map = Eigen::Vector3i();
  Eigen::Vector3i origin = occ_gridmap.origin();

  pose_map.x() = origin.x() + static_cast<int>(round(pose(0) / map_res));
  pose_map.y() = origin.y() + static_cast<int>(round(pose(1) / map_res));
  pose_map.z() = origin.z() + static_cast<int>(round(pose(2) / map_res));

  return pose_map;
};

Eigen::Vector3i world_to_map(Eigen::Matrix4d &pose, voxel_map::Map &occ_gridmap,
                             float map_res) {

  Eigen::Vector3i pose_map = Eigen::Vector3i();
  Eigen::Vector3i origin = occ_gridmap.origin();

  pose_map.x() = origin.x() + static_cast<int>(round(pose(0, 3) / map_res));
  pose_map.y() = origin.y() + static_cast<int>(round(pose(1, 3) / map_res));
  pose_map.z() = origin.z() + static_cast<int>(round(pose(2, 3) / map_res));

  return pose_map;
};

Eigen::Vector3i pose_to_voxel(Eigen::Matrix4d &pose,
                              voxel_map::Map &occ_gridmap, float map_res) {
  return world_to_map(pose, occ_gridmap, map_res);
};

std::vector<Eigen::Vector3i> cloud_to_map(Vector3dVector &cloud,
                                          Eigen::Matrix4d &pose,
                                          voxel_map::Map &occ_gridmap,
                                          float map_res) {

  std::vector<Eigen::Vector3i> cloud_map = std::vector<Eigen::Vector3i>();
  Eigen::Matrix3d R = pose.block(0, 0, 3, 3);
  Eigen::Vector3d t = pose.col(3).head(3);
  // std::cout << t << " ";

  for (auto p : cloud) {
    Eigen::Vector3d tr = R * p + t;
    Eigen::Vector3i pose_map = world_to_map(tr, occ_gridmap, map_res);
    // std::cout << pose_map << " ";
    cloud_map.push_back(pose_map);
  }
  // Vector3dVector transformed_cloud = R * cloud;

  return cloud_map;
};

int main() {

  const dataloader::Dataset dataset = Dataset("Data");

  std::cout << "dataset.size: " << dataset.size() << std::endl;

  // 1. init map here
  // occ_gridmap()
  float map_res = 0.01;
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
  for (int i = 0; i < dataset.size(); i++) {
    const PoseAndCloud pose_and_cloud = dataset[i];
    auto pose = pose_and_cloud.first;
    auto cloud = pose_and_cloud.second;
    Eigen::Vector3i pose_on_map = pose_to_voxel(pose, occ_gridmap, map_res);
    std::vector<Eigen::Vector3i> clouds_on_map =
        cloud_to_map(cloud, pose, occ_gridmap, map_res);

    for (int j = 0; j < clouds_on_map.size(); j++) {
      bresenham::Bresenham(pose_on_map, clouds_on_map[j]);
    }
    std::cout << "frame id: " << i << std::endl;
    std::cout << "pose_on_map : " << pose_on_map << std::endl;
    std::cout << "cloud size : " << cloud.size() << std::endl;
    std::cout << "cloud_on_map size : " << clouds_on_map.size() << std::endl;
    std::cout << "pose : " << pose << std::endl;

    // grid_mapping_with_known_poses()
  }

  // 3. visualize(occ_gridmap);
  return 0;
}
#include "map.hpp"
#include "bresenham.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// namespace fs = std::filesystem;

namespace {

struct CompareEigenPoints {
  bool operator()(const Eigen::Vector3i &a, const Eigen::Vector3i &b) const {
    return std::lexicographical_compare(a.data(), a.data() + 3, b.data(),
                                        b.data() + 3);
  }
};

float prob2logodds(float p) {
  if (p == 1) {
    return 0;
  }
  return log(p / (1 - p));
}

float logodds2prob(float l) { return 1 - (1 / (1 + exp(l))); }

float inverse_sensor_model(float p) { return prob2logodds(p); };

} // namespace

namespace voxel_map {
G_Map::G_Map(const float size_, const float res_) : size(size_), res(res_) {
  double shape_f = round(size_ / res_);
  shape = static_cast<int>(round(size_ / res_));
}

G_Map::G_Map(const float size_, const float res_, const float prob_occ_,
             const float prob_free_, const float prior_)
    : size(size_), res(res_), prob_occ(prob_occ_), prob_free(prob_free_),
      prior(prior_) {
  double shape_f = round(size_ / res_);
  shape = static_cast<int>(round(size_ / res_));
}

void G_Map::Voxelization(Eigen::Matrix4d &pose, const Vector3dVector &cloud) {

  Eigen::Vector3i pose_map_ = pose_to_voxel(pose);
  auto it = voxels_.find(pose_map_);

  // found
  // if (it != voxels_.end()) {
  //   return;
  // } else {
  // std::cout << "not found" << std::endl;
  cloud_to_map(pose, cloud);
  // }
}

void G_Map::Voxelization(Eigen::Matrix4d &pose) {

  Voxel vox;
  Eigen::Vector3i pose_map_ = pose_to_voxel(pose);

  vox.index_ = pose_map_;
  vox.point_ = pose.col(3).head(3);

  auto it = voxels_.find(pose_map_);

  // if (it == voxels_.end()) // not found
  voxels_.insert({pose_map_, vox});
}

std::vector<Eigen::Vector3d> &G_Map::GetPointCloud() {

  for (auto v : voxels_) {
    if (v.second.prop >= prob_occ) {
      points_.emplace_back(v.second.point_);
    }
  };

  return points_;
}

void G_Map::cloud_to_map(Eigen::Matrix4d &pose, const Vector3dVector &cloud) {

  Eigen::Matrix3d R = pose.block(0, 0, 3, 3);
  Eigen::Vector3d t = pose.col(3).head(3);

  Vector3dVector transformed_pcd;
  transformed_pcd.resize(cloud.size());

  std::transform(cloud.begin(), cloud.end(), transformed_pcd.begin(),
                 [&R, &t](const Eigen::Vector3d &p) { return R * p + t; });

  for (int i = 0; i < transformed_pcd.size(); i++) {
    Eigen::Vector3d pointcloud = transformed_pcd[i];
    Eigen::Vector3i pointcloud_index = Eigen::Vector3i();
    pointcloud_index = world_to_map(pointcloud_index, pointcloud);

    auto it = voxels_.find(pointcloud_index);

    if (it == voxels_.end()) // not found or free
    {

      Eigen::Vector3i translation = pose_map_;
      Eigen::Vector3i pointcloud_index_translation =
          (pointcloud_index - translation);
      bresenham::Bresenham bresenham =
          bresenham::Bresenham(translation, pointcloud_index_translation);
      std::vector<Eigen::Vector3i> coordinates_int = bresenham.coordinates_int;

      for (auto coor : coordinates_int) {

        auto it = voxels_.find(coor);
        if (it == voxels_.end()) // not found but free
        {
          Voxel vox_between;
          vox_between.index_ = coor;
          Eigen::Vector3d coor_3d = Eigen::Vector3d();

          coor_3d.x() = static_cast<double>(res * (coor(0, 0)));
          coor_3d.y() = static_cast<double>(res * (coor(1, 0)));
          coor_3d.z() = static_cast<double>(res * (coor(2, 0)));
          vox_between.point_ = coor_3d;
          float prev_prop = prob2logodds(prior);
          // init the free space, the prop prop here is 0.35;
          vox_between.prop =
              logodds2prob(prev_prop + inverse_sensor_model(prob_free) -
                           prob2logodds(prior));
          voxels_.insert({coor, vox_between});

        } else { // FOUND but free
          // update the free space, the prop of free space go lower;
          float prev_prop = prob2logodds(it->second.prop);
          it->second.prop =
              logodds2prob(prev_prop + inverse_sensor_model(prob_free) -
                           prob2logodds(prior));
          voxels_.insert({coor, it->second});
        }
      }

      Voxel vox_end;
      vox_end.index_ = pointcloud_index;
      vox_end.point_ = pointcloud;
      // vox_end
      float prev_prop = prob2logodds(prior);
      vox_end.prop = logodds2prob(prev_prop + inverse_sensor_model(prob_occ) -
                                  prob2logodds(prior));

      voxels_.insert({pointcloud_index, vox_end});

    } else {
      // FOUND for end point
      // vox_end
      float prev_prop = prob2logodds(it->second.prop);
      it->second.prop = logodds2prob(
          prev_prop + inverse_sensor_model(prob_occ) - prob2logodds(prior));
    }
  }
};

Eigen::Vector3i G_Map::world_to_map(Eigen::Vector3i &pointcloud_index,
                                    Eigen::Vector3d &pointcloud) {

  pointcloud_index.x() =
      pose_map_.x() + static_cast<int>(round(pointcloud(0) / res));
  pointcloud_index.y() =
      pose_map_.y() + static_cast<int>(round(pointcloud(1) / res));
  pointcloud_index.z() =
      pose_map_.z() + static_cast<int>(round(pointcloud(2) / res));

  return pointcloud_index;
};

Eigen::Vector3i G_Map::world_to_map(Eigen::Matrix4d &pose) {

  pose_map_.x() = static_cast<int>(round(pose(0, 3) / res));
  pose_map_.y() = static_cast<int>(round(pose(1, 3) / res));
  pose_map_.z() = static_cast<int>(round(pose(2, 3) / res));

  return pose_map_;
};

Eigen::Vector3i G_Map::pose_to_voxel(Eigen::Matrix4d &pose) {
  return world_to_map(pose);
};

} // namespace voxel_map
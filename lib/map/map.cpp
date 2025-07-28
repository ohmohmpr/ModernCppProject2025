#include "map.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

// namespace fs = std::filesystem;

// namespace {

// std::vector<std::string> ReadPointCloudFiles(const fs::path &pointcloud_path)
// {
//   std::vector<std::string> filenames;
//   for (const auto &file : fs::directory_iterator(pointcloud_path)) {
//     if (file.path().extension() == ".ply") {
//       filenames.emplace_back(file.path().string());
//     }
//   }
//   if (filenames.empty()) {
//     std::cerr << pointcloud_path << "contains no files with .ply extension"
//               << std::endl;
//     exit(1);
//   }
//   std::sort(filenames.begin(), filenames.end());

//   return filenames;
// }

// Vector3dVector ExtractPointCloud(const std::string &filename) {
//   open3d::geometry::PointCloud pointcloud;
//   open3d::io::ReadPointCloudFromPLY(filename, pointcloud,
//                                     open3d::io::ReadPointCloudOption());
//   return pointcloud.points_;
// }

// std::vector<Eigen::Matrix4d> ReadPoses(const std::string &filename) {
//   std::ifstream file(filename);
//   std::vector<Eigen::Matrix4d> poses;
//   Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
//   while (file >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3) >>
//          pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3) >> pose(2, 0)
//          >> pose(2, 1) >> pose(2, 2) >> pose(2, 3)) {
//     poses.emplace_back(pose);
//   }
//   return poses;
// }

// } // namespace

namespace voxel_map {
Map::Map(const float size_, const float res_) : size(size_), res(res_) {
  shape = static_cast<int>(round(size_ / res_));
  origin_ = {shape / 2, shape / 2, shape / 2};
  Vector3dVector voxels_ = std::vector<Eigen::Vector3d>();
  voxels_.reserve(shape);
  //   std::cout << "number_cells: " << number_cells << std::endl;
  //   std::cout << "voxels_.size: " << voxels_.max_size() << std::endl;
}

// Dataset::Dataset(const std::string& data_dir) {
//     pointcloud_files_ = ReadPointCloudFiles(fs::path(data_dir) / "PLY/");

//     poses_.reserve(pointcloud_files_.size());
//     poses_ = ReadPoses(fs::path(data_dir) / "gt_poses.txt");
// }

// PoseAndCloud Dataset::operator[](
//         const int idx) const {
//     return std::make_pair(poses_[idx],
//     ExtractPointCloud(pointcloud_files_[idx]));
// }
} // namespace voxel_map
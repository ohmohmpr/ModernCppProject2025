#include "bresenham.hpp"

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {

int test_bresenham(int i, int start, int end) {

  if (end >= start) {
    if (i < start) {
      return 1;
    }
    if (i > end) {
      return 1;
    }
  } else {
    if (i > start) {
      return 1;
    }
    if (i < end) {
      return 1;
    }
  }

  return 0;
}

int determine_step(int start, int end) {

  if (start > end) {
    return -1;
  }
  return 1;
}

int find_diving_axis(Eigen::Vector3i &pose_on_map,
                     Eigen::Vector3i &cloud_on_map) {

  int x_start = pose_on_map.x();
  int y_start = pose_on_map.y();
  int z_start = pose_on_map.z();
  int x_end = cloud_on_map.x();
  int y_end = cloud_on_map.y();
  int z_end = cloud_on_map.z();

  int x_abs = std::abs(x_end - x_start);
  int y_abs = std::abs(y_end - y_start);
  int z_abs = std::abs(z_end - z_start);

  int axis_index = 0;
  int xyz_abs = std::max({x_abs, y_abs, z_abs});
  if (xyz_abs == x_abs) {
    axis_index = 0;
  } else if (xyz_abs == y_abs) {
    axis_index = 1;
  } else if (xyz_abs == z_abs) {
    axis_index = 2;
  };

  return axis_index;
}

} // namespace

namespace bresenham {
Bresenham::Bresenham(Eigen::Vector3i &pose_on_map,
                     Eigen::Vector3i &cloud_on_map) {

  int x_start = pose_on_map.x();
  int y_start = pose_on_map.y();
  int z_start = pose_on_map.z();

  int x_end = cloud_on_map.x();
  int y_end = cloud_on_map.y();
  int z_end = cloud_on_map.z();

  diving_axis_ = find_diving_axis(pose_on_map, cloud_on_map);
  if (diving_axis_ == 0) {
    coordinates_int.push_back({x_start, y_start, z_start});
    find_coordinates(x_start, y_start, z_start, x_end, y_end, z_end,
                     coordinates_int);
  } else if (diving_axis_ == 1) {
    coordinates_int.push_back({y_start, x_start, z_start});
    find_coordinates(y_start, x_start, z_start, y_end, x_end, z_end,
                     coordinates_int);
  } else if (diving_axis_ == 2) {
    coordinates_int.push_back({z_start, x_start, y_start});
    find_coordinates(z_start, x_start, y_start, z_end, x_end, y_end,
                     coordinates_int);
  };
}

void Bresenham::find_coordinates(
    int main_start, int second_axis_start, int third_axis_start, int main_end,
    int second_axis_end, int third_axis_end,
    std::vector<Eigen::Vector3i> &coordinates_int) {

  int main_i = main_start;
  int second_axis_i = second_axis_start;
  int third_axis_i = third_axis_start;

  int delta_main = std::abs(main_end - main_start);
  int delta_second = std::abs(second_axis_end - second_axis_start);
  int delta_third = std::abs(third_axis_end - third_axis_start);

  int main_step = determine_step(main_start, main_end);
  int second_step = determine_step(second_axis_start, second_axis_end);
  int third_step = determine_step(third_axis_start, third_axis_end);

  int p_second = 2 * delta_second - delta_main;
  int p_third = 2 * delta_third - delta_main;

  while (main_i != main_end) {
    main_i = main_i + main_step;
    if (p_second > 0) {
      second_axis_i = second_axis_i + second_step;
      p_second = p_second - 2 * delta_main;
    }
    if (p_third > 0) {
      third_axis_i = third_axis_i + third_step;
      p_third = p_third - 2 * delta_main;
    }
    p_second = p_second + 2 * delta_second;
    p_third = p_third + 2 * delta_third;

    // for testing the algorithm.
    // if (test_bresenham(main_i, main_start, main_end) == 1) {
    //   std::cerr << "main_i error" << std::endl;
    //   std::cout << "main_start: " << main_start << std::endl;
    //   std::cout << "main_i: " << main_i << std::endl;
    //   std::cout << "main_end: " << main_end << std::endl;
    //   break;
    // }
    // if (test_bresenham(second_axis_i, second_axis_start, second_axis_end) ==
    //     1) {
    //   std::cerr << "second_axis_i error" << std::endl;
    //   std::cout << "second_axis_start: " << second_axis_start << std::endl;
    //   std::cout << "second_axis_i: " << second_axis_i << std::endl;
    //   std::cout << "second_axis_end: " << second_axis_end << std::endl;
    //   break;
    // }
    // if (test_bresenham(third_axis_i, third_axis_start, third_axis_end) == 1) {
    //   std::cerr << "third_axis_i error" << std::endl;
    //   std::cout << "third_axis_start: " << third_axis_start << std::endl;
    //   std::cout << "third_axis_i: " << third_axis_i << std::endl;
    //   std::cout << "third_axis_end: " << third_axis_end << std::endl;
    //   break;
    // }

    if (diving_axis_ == 0) {
      coordinates_int.push_back({main_i, second_axis_i, third_axis_i});
    } else if (diving_axis_ == 1) {
      coordinates_int.push_back({second_axis_i, main_i, third_axis_i});
    } else if (diving_axis_ == 2) {
      coordinates_int.push_back({second_axis_i, third_axis_i, main_i});
    };
  }
}

} // namespace bresenham

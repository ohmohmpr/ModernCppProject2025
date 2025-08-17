cmake_minimum_required(VERSION 3.22)
project(grid_mapping LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# Locate Eigen3 Library dependency
find_package(Eigen3 3.4 REQUIRED)
# Locate Open3D Library dependency
find_package(Open3D REQUIRED HINTS ${PROJECT_SOURCE_DIR}/3rdparty/open3d/lib/cmake)

add_library(dataloader)
target_sources(dataloader PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/dataloader/dataloader.cpp)
target_link_libraries(dataloader PUBLIC Eigen3::Eigen Open3D::Open3D)
target_include_directories(dataloader PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/dataloader)

add_library(bresenham)
target_sources(bresenham PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/bresenham/bresenham.cpp)
target_link_libraries(bresenham PUBLIC Eigen3::Eigen Open3D::Open3D)
target_include_directories(bresenham PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/bresenham)

add_library(map)
target_sources(map PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/map/map.cpp)
target_link_libraries(map PUBLIC Eigen3::Eigen Open3D::Open3D bresenham)
target_include_directories(map PUBLIC ${CMAKE_CURRENT_LIST_DIR}/lib/map)

include_directories(include/visualizer)
include_directories(include)

add_executable(grid_mapping)
target_sources(grid_mapping PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)
target_link_libraries(grid_mapping PRIVATE dataloader map)
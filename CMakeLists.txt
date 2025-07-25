cmake_minimum_required(VERSION 3.22)
project(grid_mapping LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


add_executable(grid_mapping)
target_sources(grid_mapping PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)
# target_link_libraries(grid_mapping )
# target_include_directories(grid_mapping PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp)
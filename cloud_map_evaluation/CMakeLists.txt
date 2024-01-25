cmake_minimum_required(VERSION 3.12.0)
project(cloud_map_evaluation)

set(CMAKE_CXX_STANDARD 17)

find_package(Open3D 0.17.0 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
)

add_executable(cloud_map_eval src/cloud_map_eval.cpp src/cloud_map_eval_node.cpp)
target_link_libraries(cloud_map_eval ${Open3D_LIBRARIES})

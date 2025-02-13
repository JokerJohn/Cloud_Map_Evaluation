cmake_minimum_required(VERSION 3.12.0)
project(map_eval)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
)

add_executable(map_eval src/map_eval.cpp src/map_eval_main.cpp src/voxel_calculator.cpp)
target_link_libraries(map_eval ${Open3D_LIBRARIES} ${PCL_LIBRARIES}  ${YAML_CPP_LIBRARIES})

# If OpenMP is found, add the flags for OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(map_eval OpenMP::OpenMP_CXX)
endif()
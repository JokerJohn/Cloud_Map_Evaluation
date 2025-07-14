cmake_minimum_required(VERSION 3.12.0)
project(map_eval)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(Open3D_DIR "/home/xchu/xicp_ws/devel/.private/open3d_catkin/lib/cmake/Open3D")
message(STATUS "LTLoc: Set Open3D_DIR to ${Open3D_DIR}") # 打印信息，确认设置生效

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "yaml-cpp library: ${YAML_CPP_LIBRARY}")
message(STATUS "yaml-cpp include: ${YAML_CPP_INCLUDE_DIR}")
# 打印信息用于调试
message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")
message(STATUS "Open3D libraries: ${Open3D_LIBRARIES}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")


include_directories(
        include
        ${Open3D_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
)

add_executable(map_eval src/map_eval.cpp src/map_eval_main.cpp src/voxel_calculator.cpp)
target_link_libraries(map_eval ${Open3D_LIBRARIES} ${PCL_LIBRARIES}  ${YAML_CPP_LIBRARY})



# If OpenMP is found, add the flags for OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(map_eval OpenMP::OpenMP_CXX)
endif()
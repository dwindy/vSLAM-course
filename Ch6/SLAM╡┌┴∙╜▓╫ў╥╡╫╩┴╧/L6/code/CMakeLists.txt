cmake_minimum_required(VERSION 3.0.0)
project(homework)

set(CMAKE_BUILD_TYPE "Release")
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(optical_flow_disparity optical_flow_disparity.cpp)
target_link_libraries(optical_flow_disparity ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES})

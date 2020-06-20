cmake_minimum_required(VERSION 3.0.0)
project(ch5 VERSION 0.1.0)

set(CMAKE_CXX_FLAGS "-std=c++11 -o2 ${SSE_FLAGS} -msse4")

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(orb_calc computeORB.cpp)
target_link_libraries(orb_calc ${OpenCV_LIBRARIES})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES})

add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA ${Sophus_LIBRARIES})

add_executable(jrajectory_aligment jrajectory_aligment.cpp)
target_link_libraries(jrajectory_aligment ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

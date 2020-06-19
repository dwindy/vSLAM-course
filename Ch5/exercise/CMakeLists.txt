cmake_minimum_required(VERSION 3.0.0)
project(orb_cv VERSION 0.1.0)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

find_package(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})

add_executable(orb_cv orb_demo.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBRARIES})
add_executable(orb_cv_mannu orb_mannu.cpp)
target_link_libraries(orb_cv_mannu ${OpenCV_LIBRARIES})

add_executable(epipolar epipolar_solution.cpp)
target_link_libraries(epipolar ${OpenCV_LIBRARIES})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBRARIES})

add_executable(Rt_3d_2d pose_estimation_3d2d.cpp)
target_link_libraries(Rt_3d_2d ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} g2o_core g2o_stuff)
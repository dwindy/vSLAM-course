cmake_minimum_required(VERSION 3.0.0)
project(homework4 VERSION 0.1.0)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
# Eigen
include_directories("/usr/include/eigen3")

add_executable(undistortion undistort_image.cpp)
target_link_libraries(undistortion ${OpenCV_LIBS})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS})

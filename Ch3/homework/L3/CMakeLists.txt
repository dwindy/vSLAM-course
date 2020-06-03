cmake_minimum_required( VERSION 2.8 )
project( code_homework )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( Sophus REQUIRED )
find_package( Pangolin REQUIRED )
find_package( Eigen3 REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

add_executable( code_homework src/draw_trajectory.cpp)
target_link_libraries( code_homework ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})
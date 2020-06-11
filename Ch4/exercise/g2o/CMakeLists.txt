cmake_minimum_required(VERSION 3.0.0)
project(hellog2o VERSION 0.1.0)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# Eigen
include_directories("/usr/include/eigen3")
# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(hellog2o main.cpp)
target_link_libraries(hellog2o ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})



cmake_minimum_required(VERSION 3.0.0)
project(gaussianNewton VERSION 0.1.0)

add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(gaussianNewton gaussNewton.cpp)
target_link_libraries(gaussianNewton ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

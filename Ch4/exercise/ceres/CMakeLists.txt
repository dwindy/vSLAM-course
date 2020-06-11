cmake_minimum_required(VERSION 3.0.0)
project(helloceres VERSION 0.1.0)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})

add_executable(helloceres ceresCurveFitting.cpp)
target_link_libraries(helloceres ${CERES_LIBRARIES} ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

cmake_minimum_required(VERSION 2.8)

project(CH7)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
#Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(OpenCV REQUIRED)
find_package( Pangolin )

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_types_sba)


include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories( ${Sophus_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories(${G2O_INCLUDE_DIRS})
#add_library(bal_common common.cpp)
#add_executable(bundle_adjustment_bal bundle_adjustment_bal.cpp)
#target_link_libraries(bundle_adjustment_bal ${G2O_LIBS} bal_common ${Sophus_LIBRARIES})

add_executable(directBA directBA.cpp)
target_link_libraries(directBA ${G2O_LIBS} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES} ${CSPARSE_LIB})
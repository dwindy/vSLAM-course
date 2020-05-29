cmake_minimum_required(VERSION 3.0.0)
project(matrix_decomposition VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(matrix_decomposition main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

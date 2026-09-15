cmake_minimum_required(VERSION 3.20)

include(FetchContent)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
FetchContent_MakeAvailable(eigen)

project(robomath)

add_library(robomath_core src/core/so2.cpp)
target_include_directories(robomath_core PUBLIC include)
target_link_libraries(robomath_core PUBLIC Eigen3::Eigen)

add_executable(test_so2 test/test_so2.cpp)
target_link_libraries(test_so2 PRIVATE robomath_core)

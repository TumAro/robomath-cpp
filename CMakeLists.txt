cmake_minimum_required(VERSION 3.20)
project(robomath VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(FetchEigen)

# groups/, tangent/ and spatial/ sources join this list as they get written
add_library(robomath_core
  src/core/so2.cpp
  src/core/se2.cpp
  src/core/so3.cpp
  src/core/se3.cpp
  src/kinematics/forward.cpp
  src/c_space.cpp
)
target_include_directories(robomath_core PUBLIC include)
target_link_libraries(robomath_core PUBLIC Eigen3::Eigen)

foreach(suite so2 kinematics spatial c_space)
  add_executable(test_${suite} tests/test_${suite}.cpp)
  target_link_libraries(test_${suite} PRIVATE robomath_core)
endforeach()

include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)

set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
FetchContent_MakeAvailable(eigen)

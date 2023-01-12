cmake_minimum_required(VERSION 3.10)
project(zstd_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
# We need at least VERSION 1.4.8, version check is done in Findzstd.cmake
find_package(zstd 1.4.8 QUIET)

ament_vendor(zstd_vendor
  SATISFIED ${zstd_FOUND}
  VCS_URL https://github.com/facebook/zstd.git
  VCS_VERSION v1.4.8
  SOURCE_SUBDIR build/cmake
  CMAKE_ARGS
    -DZSTD_BUILD_STATIC:BOOL=OFF
    -DZSTD_BUILD_SHARED:BOOL=ON
    -DZSTD_BUILD_PROGRAMS:BOOL=OFF
)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS zstd_vendor-extras.cmake)

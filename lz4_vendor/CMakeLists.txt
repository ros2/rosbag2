cmake_minimum_required(VERSION 3.10)
project(lz4_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

ament_add_default_options()

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(lz4 QUIET)

ament_vendor(lz4_vendor
  SATISFIED ${lz4_FOUND}
  VCS_URL https://github.com/lz4/lz4.git
  VCS_VERSION v1.9.4
  SOURCE_SUBDIR build/cmake
  CMAKE_ARGS
    -DBUILD_STATIC_LIBS:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DLZ4_BUILD_LEGACY_LZ4C:BOOL=OFF
    -DLZ4_BUILD_CLI:BOOL=OFF
)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS lz4_vendor-extras.cmake)

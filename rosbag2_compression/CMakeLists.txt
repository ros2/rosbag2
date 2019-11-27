cmake_minimum_required(VERSION 3.5)
project(rosbag2_compression)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(DISABLE_SANITIZERS "Disable the use of gcc sanitizers")
# Only enable sanitizers on x64 architectures
# https://github.com/google/sanitizers/issues/794
if(NOT ${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(DISABLE_SANITIZERS ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_library(ZSTD_LIBRARY NAMES zstd)  # Assumes libzstd-dev is installed

add_library(${PROJECT_NAME}_zstd SHARED
  src/rosbag2_compression/zstd_compressor.cpp)
target_include_directories(${PROJECT_NAME}_zstd PUBLIC include)
target_link_libraries(${PROJECT_NAME}_zstd ${ZSTD_LIBRARY})
ament_target_dependencies(${PROJECT_NAME}_zstd
  rcpputils
  rcutils
  rosbag2_storage)

install(
  DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_library(fake_compressor SHARED
    test/rosbag2_compression/fake_compressor.cpp)
  target_include_directories(fake_compressor PUBLIC include)
  ament_target_dependencies(fake_compressor rosbag2_storage)
  set_target_properties(fake_compressor
    PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1)
  target_compile_definitions(fake_compressor PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

  ament_add_gmock(test_compressor
    test/rosbag2_compression/test_compressor.cpp)
  target_link_libraries(test_compressor fake_compressor)
  ament_target_dependencies(test_compressor rcpputils)

  add_library(fake_decompressor SHARED
    test/rosbag2_compression/fake_decompressor.cpp)
  target_include_directories(fake_decompressor PUBLIC include)
  ament_target_dependencies(fake_decompressor rosbag2_storage rcpputils)
  set_target_properties(fake_decompressor
    PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1)
  target_compile_definitions(fake_decompressor PRIVATE ROSBAG2_COMPRESSION_BUILDING_DLL)

  ament_add_gmock(test_decompressor
    test/rosbag2_compression/test_decompressor.cpp)
  target_link_libraries(test_decompressor fake_decompressor)

  ament_add_gmock(test_zstd_compressor
    test/rosbag2_compression/test_zstd_compressor.cpp)
  target_include_directories(test_zstd_compressor PUBLIC include)
  target_link_libraries(test_zstd_compressor ${PROJECT_NAME}_zstd)
  ament_target_dependencies(test_zstd_compressor rosbag2_test_common rosbag2_storage)
endif()

ament_package()

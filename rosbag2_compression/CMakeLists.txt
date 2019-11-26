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

option(DISABLE_SANITIZERS "disables the use of gcc saniztizers")
# Only enable sanitizers on x64 architectures
# https://github.com/google/sanitizers/issues/794
if(NOT ${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(DISABLE_SANITIZERS ON)
endif()

find_library(ZSTD_LIBRARY NAMES zstd)  # Assume libzstd-dev is installed
find_package(ament_cmake REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

add_library(${PROJECT_NAME}_zstd SHARED
  src/rosbag2_compression/zstd_compressor.cpp)
target_include_directories(${PROJECT_NAME}_zstd PUBLIC include)
target_link_libraries(${PROJECT_NAME}_zstd
  ${ZSTD_LIBRARY}
  ${rosbag2_storage_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rcpputils_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_zstd
  rosbag2_storage
  rcpputils
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_zstd_compression
    test/rosbag2_compression/test_zstd_compression.cpp)
  if(TARGET test_zstd_compression)
    target_include_directories(test_zstd_compression PUBLIC include)
    target_link_libraries(test_zstd_compression
      ${PROJECT_NAME}_zstd
      ${rosbag2_storage_LIBRARIES}
      ${rosbag2_test_common_LIBRARIES}
      )
    ament_target_dependencies(test_zstd_compression rosbag2_test_common rosbag2_storage)
  endif()
endif()

ament_package()
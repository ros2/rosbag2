cmake_minimum_required(VERSION 3.5)
project(rosbag2_compression_zstd)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(zstd_vendor REQUIRED)
find_package(zstd REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_compression_zstd/compression_utils.cpp
  src/rosbag2_compression_zstd/zstd_compressor.cpp
  src/rosbag2_compression_zstd/zstd_decompressor.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}
  rosbag2_compression::rosbag2_compression
  zstd::zstd
)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROSBAG2_COMPRESSION_ZSTD_BUILDING_DLL)
pluginlib_export_plugin_description_file(rosbag2_compression plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

# order matters here, first vendor, then zstd
ament_export_dependencies(rosbag2_compression zstd_vendor zstd)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_zstd_compressor
    test/rosbag2_compression_zstd/test_zstd_compressor.cpp)
  target_link_libraries(test_zstd_compressor
    ${PROJECT_NAME}
    rclcpp::rclcpp
    rosbag2_test_common::rosbag2_test_common
  )
endif()

ament_package()

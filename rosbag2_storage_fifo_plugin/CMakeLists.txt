cmake_minimum_required(VERSION 3.8)
project(rosbag2_storage_fifo_plugin)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_storage_mcap REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  rosbag2_storage_fifo_plugin SHARED
  src/fifo_storage.cpp
)

target_include_directories(rosbag2_storage_fifo_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  rosbag2_storage_fifo_plugin
  rclcpp
  rosbag2_storage
  rosbag2_storage_mcap
  yaml_cpp_vendor
  mcap_vendor
  pluginlib
)

install(
  TARGETS rosbag2_storage_fifo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(rosbag2_performance_benchmarking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(writer_benchmark
  src/config_utils.cpp
  src/result_utils.cpp
  src/writer_benchmark.cpp)
ament_target_dependencies(writer_benchmark
  rclcpp
  std_msgs
  rosbag2_compression
  rosbag2_cpp
  rosbag2_storage
  yaml_cpp_vendor
)
target_include_directories(writer_benchmark
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(benchmark_publishers
  src/benchmark_publishers.cpp
  src/config_utils.cpp)
ament_target_dependencies(benchmark_publishers
  rclcpp
  rosbag2_storage
  std_msgs
  yaml_cpp_vendor
)
target_include_directories(benchmark_publishers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(results_writer
  src/config_utils.cpp
  src/result_utils.cpp
  src/results_writer.cpp)
ament_target_dependencies(results_writer
  rclcpp
  rosbag2_storage
)
target_include_directories(results_writer
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(
  writer_throughput
  src/writer_throughput.cpp
)
ament_target_dependencies(
  writer_throughput
  rclcpp
  std_msgs
  rosbag2_cpp
)
target_include_directories(writer_benchmark
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(
  TARGETS writer_benchmark benchmark_publishers results_writer writer_throughput
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(rosbag2_performance_benchmarking)

if (NOT BUILD_ROSBAG2_BENCHMARKS)
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(rosbag2_performance_benchmarking_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(writer_benchmark
  src/config_utils.cpp
  src/result_utils.cpp
  src/writer_benchmark.cpp)

add_executable(benchmark_publishers
  src/benchmark_publishers.cpp
  src/config_utils.cpp
  src/msg_utils/helpers.cpp)

add_executable(results_writer
  src/config_utils.cpp
  src/result_utils.cpp
  src/results_writer.cpp)

target_link_libraries(writer_benchmark
  rclcpp::rclcpp
  ${rosbag2_performance_benchmarking_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rosbag2_compression::rosbag2_compression
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(benchmark_publishers
  rclcpp::rclcpp
  rosbag2_storage::rosbag2_storage
  ${rosbag2_performance_benchmarking_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(results_writer
  rclcpp::rclcpp
  rosbag2_storage::rosbag2_storage
)

target_include_directories(writer_benchmark
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(benchmark_publishers
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(results_writer
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS writer_benchmark benchmark_publishers results_writer
  DESTINATION lib/${PROJECT_NAME})

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

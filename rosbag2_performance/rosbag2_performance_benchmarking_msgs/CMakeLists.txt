cmake_minimum_required(VERSION 3.5)

project(rosbag2_performance_benchmarking_msgs)

option(BUILD_ROSBAG2_BENCHMARKS "Build rosbag2 performance benchmarks" OFF)

if(NOT BUILD_ROSBAG2_BENCHMARKS)
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ByteArray.msg"
  DEPENDENCIES
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

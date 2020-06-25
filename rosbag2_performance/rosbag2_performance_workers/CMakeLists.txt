cmake_minimum_required(VERSION 3.5)
project(rosbag2_performance_workers)

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

find_package(ament_cmake REQUIRED)
if(BUILD_BENCHMARKS)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(include)

  add_executable(image_worker src/image.cpp)
  ament_target_dependencies(image_worker rclcpp sensor_msgs std_msgs)

  add_executable(pointcloud2_worker src/pointcloud2.cpp)
  ament_target_dependencies(pointcloud2_worker rclcpp sensor_msgs std_msgs)

  add_executable(bytearray_worker src/bytearray.cpp)
  ament_target_dependencies(bytearray_worker rclcpp std_msgs)

  install(TARGETS image_worker pointcloud2_worker bytearray_worker
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()
ament_package()

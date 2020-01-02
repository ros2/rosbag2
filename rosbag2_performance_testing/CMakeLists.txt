cmake_minimum_required(VERSION 3.5)
project(rosbag2_performance_testing)

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

if(BUILD_PERFORMANCE)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  add_executable(image_publisher1 performance_publishers/large_message_test/src/image_publisher1.cpp)
  target_include_directories(image_publisher1 PRIVATE large_message_test/src)
  ament_target_dependencies(image_publisher1 rclcpp rclcpp_components sensor_msgs std_msgs)

  add_executable(image_publisher2 performance_publishers/large_message_test/src/image_publisher2.cpp)
  target_include_directories(image_publisher2 PRIVATE large_message_test/src)
  ament_target_dependencies(image_publisher2 rclcpp rclcpp_components sensor_msgs std_msgs)

  add_executable(pointcloud_publisher performance_publishers/large_message_test/src/point_cloud2_publisher.cpp)
  target_include_directories(pointcloud_publisher PRIVATE large_message_test/src)
  ament_target_dependencies(pointcloud_publisher rclcpp rclcpp_components sensor_msgs std_msgs)

  install(DIRECTORY performance_publishers/large_message_test/launch
    DESTINATION share/${PROJECT_NAME}/)

  install(DIRECTORY performance_publishers/large_message_test/expected
          DESTINATION share/${PROJECT_NAME}/)

  install(DIRECTORY scripts/
          DESTINATION share/${PROJECT_NAME}/)

  install(TARGETS image_publisher1 image_publisher2 pointcloud_publisher
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

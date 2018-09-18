cmake_minimum_required(VERSION 3.5)
project(rosbag2_tests)

add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/resources")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rosbag2_storage_default_plugins REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_record_end_to_end_test
    test/rosbag2_record_end_to_end_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ENV ROS_DOMAIN_ID=42) # make sure this test does not interfer with transport tests
  if(TARGET rosbag2_record_end_to_end_test)
    ament_target_dependencies(rosbag2_record_end_to_end_test
      rclcpp
      rosbag2_storage
      rosbag2_storage_default_plugins
      test_msgs)
  endif()

  ament_add_gmock(rosbag2_play_end_to_end_test
    test/rosbag2_play_end_to_end_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ENV ROS_DOMAIN_ID=42) # make sure this test does not interfer with transport tests
  if(TARGET rosbag2_play_end_to_end_test)
    ament_target_dependencies(rosbag2_play_end_to_end_test
      rclcpp
      rosbag2_storage
      rosbag2_storage_default_plugins
      test_msgs)
  endif()

endif()

ament_package()

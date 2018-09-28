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
  find_package(rosbag2_test_common REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_rosbag2_record_end_to_end
    test/rosbag2_tests/test_rosbag2_record_end_to_end.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_rosbag2_record_end_to_end)
    ament_target_dependencies(test_rosbag2_record_end_to_end
      rclcpp
      rosbag2_storage
      rosbag2_storage_default_plugins
      rosbag2_test_common
      test_msgs)
  endif()

  ament_add_gmock(test_rosbag2_play_end_to_end
    test/rosbag2_tests/test_rosbag2_play_end_to_end.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_rosbag2_play_end_to_end)
    ament_target_dependencies(test_rosbag2_play_end_to_end
      rclcpp
      rosbag2_storage
      rosbag2_storage_default_plugins
      rosbag2_test_common
      test_msgs)
  endif()

endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(rosbag2_tests)

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
  find_package(std_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_end_to_end_test
    test/rosbag2_end_to_end_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_end_to_end_test)
    ament_target_dependencies(rosbag2_end_to_end_test
      rclcpp
      std_msgs)
  endif()

endif()

ament_package()

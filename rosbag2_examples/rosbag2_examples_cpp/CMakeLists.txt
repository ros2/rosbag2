cmake_minimum_required(VERSION 3.5)
project(rosbag2_examples_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
target_link_libraries(simple_bag_recorder
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${example_interfaces_TARGETS}
)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_node src/data_generator_node.cpp)
target_link_libraries(data_generator_node
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${example_interfaces_TARGETS}
)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_executable src/data_generator_executable.cpp)
target_link_libraries(data_generator_executable
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${example_interfaces_TARGETS}
)

install(TARGETS
  data_generator_executable
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

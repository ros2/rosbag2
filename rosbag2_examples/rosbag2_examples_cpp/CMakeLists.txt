cmake_minimum_required(VERSION 3.20)
project(rosbag2_examples_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(simple_bag_recorder src/simple_bag_recorder.cpp)
target_link_libraries(simple_bag_recorder
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  example_interfaces::example_interfaces
)

install(TARGETS
  simple_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(compressed_bag_recorder src/compressed_bag_recorder.cpp)
target_link_libraries(compressed_bag_recorder
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_compression::rosbag2_compression
  example_interfaces::example_interfaces
)

install(TARGETS
  compressed_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(compressed_bag_rewriter src/compressed_bag_rewriter.cpp)
target_link_libraries(compressed_bag_rewriter
  ament_cmake_ros_core::ament_ros_defaults
  rosbag2_transport::rosbag2_transport
  example_interfaces::example_interfaces
)

install(TARGETS
  compressed_bag_rewriter
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(simple_bag_reader src/simple_bag_reader.cpp)
target_link_libraries(simple_bag_reader
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  rosbag2_transport::rosbag2_transport
  example_interfaces::example_interfaces
)

install(TARGETS
  simple_bag_reader
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_node src/data_generator_node.cpp)
target_link_libraries(data_generator_node
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  example_interfaces::example_interfaces
)

install(TARGETS
  data_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(data_generator_executable src/data_generator_executable.cpp)
target_link_libraries(data_generator_executable
  ament_cmake_ros_core::ament_ros_defaults
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  example_interfaces::example_interfaces
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

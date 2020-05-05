cmake_minimum_required(VERSION 3.5)
project(rosbag2_test_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} INTERFACE rclcpp rcutils)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp rcutils)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()

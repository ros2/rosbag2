cmake_minimum_required(VERSION 3.5)
project(rosbag2_test_common)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp::rclcpp
  rcutils::rcutils
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rcpputils REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_executable(loop_with_ctrl_c_handler test/rosbag2_test_common/loop_with_ctrl_c_handler.cpp)
  install(
      TARGETS loop_with_ctrl_c_handler
      EXPORT export_loop_with_ctrl_c_handler
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
  )

  ament_add_gmock(test_process_execution_helpers
    test/rosbag2_test_common/test_process_execution_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_process_execution_helpers)
    target_link_libraries(test_process_execution_helpers ${PROJECT_NAME})
  endif()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rclcpp rcutils)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(rosbag2)

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
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(
  librosbag
  SHARED
  src/rosbag2/rosbag2.cpp)

ament_target_dependencies(librosbag rclcpp rcutils std_msgs rosbag2_storage)
target_include_directories(librosbag
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_record src/rosbag2/demo_record.cpp)
target_link_libraries(${PROJECT_NAME}_record librosbag)

add_executable(${PROJECT_NAME}_play src/rosbag2/demo_play.cpp)
target_link_libraries(${PROJECT_NAME}_play librosbag)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS librosbag
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_write_integration_test
    test/rosbag2/rosbag2_write_integration_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_write_integration_test)
    target_link_libraries(rosbag2_write_integration_test librosbag)
  endif()

  ament_add_gmock(rosbag2_read_integration_test
    test/rosbag2/rosbag2_read_integration_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_read_integration_test)
    target_link_libraries(rosbag2_read_integration_test librosbag)
  endif()
endif()

ament_package()

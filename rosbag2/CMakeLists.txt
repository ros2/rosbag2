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
find_package(ament_index_cpp REQUIRED)
find_package(poco_vendor)
find_package(Poco COMPONENTS Foundation)
find_package(rcl REQUIRED)
find_package(rosidl_generator_c REQUIRED)

add_library(
  librosbag2
  SHARED
  src/rosbag2/rosbag2.cpp
  src/rosbag2/typesupport_helpers.cpp)

ament_target_dependencies(librosbag2 rclcpp rcutils std_msgs rcl Poco ament_index_cpp rosbag2_storage)
target_include_directories(librosbag2
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(librosbag2 PRIVATE "ROSBAG2_BUILDING_DLL")

add_executable(${PROJECT_NAME}_record src/rosbag2/demo_record.cpp)
target_link_libraries(${PROJECT_NAME}_record librosbag2)

add_executable(${PROJECT_NAME}_play src/rosbag2/demo_play.cpp)
target_link_libraries(${PROJECT_NAME}_play librosbag2)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS librosbag2
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
    target_link_libraries(rosbag2_write_integration_test librosbag2)
  endif()

  ament_add_gmock(rosbag2_read_integration_test
    test/rosbag2/rosbag2_read_integration_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_read_integration_test)
    target_link_libraries(rosbag2_read_integration_test librosbag2)
  endif()

  ament_add_gmock(rosbag2_typesupport_helpers_test
    test/rosbag2/rosbag2_typesupport_helpers_test.cpp
    src/rosbag2/typesupport_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_typesupport_helpers_test)
    ament_target_dependencies(rosbag2_typesupport_helpers_test rcl Poco ament_index_cpp
      rosidl_generator_c)
  endif()
endif()

ament_package()

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
find_package(ament_index_cpp REQUIRED)
find_package(poco_vendor)
find_package(Poco COMPONENTS Foundation)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  librosbag2
  SHARED
  src/rosbag2/rosbag2.cpp
  src/rosbag2/typesupport_helpers.cpp
  src/rosbag2/generic_publisher.cpp
  src/rosbag2/generic_subscription.cpp
  src/rosbag2/rosbag2_node.cpp)

ament_target_dependencies(
  librosbag2
  ament_index_cpp
  Poco
  rcl
  rclcpp
  rcutils
  rosbag2_storage
  rosidl_generator_c
  std_msgs
)
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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosbag2_storage_default_plugins REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_write_integration_test
    test/rosbag2/rosbag2_write_integration_test.cpp
    test/rosbag2/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_write_integration_test)
    target_link_libraries(rosbag2_write_integration_test librosbag2
      ${rosbag2_storage_default_plugins_LIBRARIES})
    target_include_directories(rosbag2_write_integration_test PRIVATE
      ${rosbag2_storage_default_plugins_INCLUDE_DIRS})
  endif()

  ament_add_gmock(rosbag2_write_all_integration_test
    test/rosbag2/rosbag2_write_all_integration_test.cpp
    test/rosbag2/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_write_all_integration_test)
    target_link_libraries(rosbag2_write_all_integration_test librosbag2
      ${rosbag2_storage_default_plugins_LIBRARIES})
    target_include_directories(rosbag2_write_all_integration_test PRIVATE
      ${rosbag2_storage_default_plugins_INCLUDE_DIRS})
  endif()

  ament_add_gmock(rosbag2_read_integration_test
    test/rosbag2/rosbag2_read_integration_test.cpp
    test/rosbag2/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_read_integration_test)
    target_link_libraries(rosbag2_read_integration_test librosbag2)
    ament_target_dependencies(rosbag2_read_integration_test
      test_msgs
    )
  endif()

  ament_add_gmock(rosbag2_play_timing_integration_test
    test/rosbag2/rosbag2_play_timing_integration_test.cpp
    test/rosbag2/test_memory_management.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_play_timing_integration_test)
    target_link_libraries(rosbag2_play_timing_integration_test librosbag2)
  endif()

  ament_add_gmock(rosbag2_typesupport_helpers_test
    test/rosbag2/rosbag2_typesupport_helpers_test.cpp
    src/rosbag2/typesupport_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_typesupport_helpers_test)
    ament_target_dependencies(rosbag2_typesupport_helpers_test
      ament_index_cpp
      Poco
      rcl
      rosidl_generator_cpp
    )
  endif()

  ament_add_gmock(rosbag2_rosbag_node_test
    test/rosbag2/rosbag2_rosbag_node_test.cpp
    test/rosbag2/test_memory_management.cpp
    src/rosbag2/generic_publisher.cpp
    src/rosbag2/generic_subscription.cpp
    src/rosbag2/rosbag2_node.cpp
    src/rosbag2/typesupport_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_rosbag_node_test)
    target_include_directories(rosbag2_rosbag_node_test
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    ament_target_dependencies(rosbag2_rosbag_node_test
      ament_index_cpp
      Poco
      rclcpp
      std_msgs
    )
  endif()
endif()

ament_package()

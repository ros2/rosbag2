cmake_minimum_required(VERSION 3.5)
project(rosbag2_storage)

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
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)

add_library(
  rosbag2_storage
  SHARED
  src/rosbag2_storage/ros_helper.cpp
  src/rosbag2_storage/storage_factory.cpp)
target_include_directories(rosbag2_storage PUBLIC include)
ament_target_dependencies(
  rosbag2_storage
  pluginlib
  rcutils)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosbag2_storage PRIVATE "ROSBAG2_STORAGE_BUILDING_DLL")

# prevent pluginlib from using boost
target_compile_definitions(rosbag2_storage PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS rosbag2_storage
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(rosbag2_test_common REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_library(
    test_plugin
    SHARED
    test/rosbag2_storage/test_plugin.cpp
    test/rosbag2_storage/test_read_only_plugin.cpp)
  target_link_libraries(test_plugin rosbag2_storage)
  install(
    TARGETS test_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  pluginlib_export_plugin_description_file(rosbag2_storage test/rosbag2_storage/test_plugin.xml)

  ament_add_gmock(test_storage_factory
    test/rosbag2_storage/test_storage_factory.cpp)
  if(TARGET test_storage_factory)
    target_include_directories(test_storage_factory PRIVATE include)
    target_link_libraries(test_storage_factory rosbag2_storage)
  endif()

  ament_add_gmock(test_ros_helper
    test/rosbag2_storage/test_ros_helper.cpp)
  if(TARGET test_ros_helper)
    target_include_directories(test_ros_helper PRIVATE include)
    target_link_libraries(test_ros_helper rosbag2_storage)
  endif()

  ament_add_gmock(test_filesystem_helper
    test/rosbag2_storage/test_filesystem_helper.cpp)
  if(TARGET test_filesystem_helper)
    target_include_directories(test_filesystem_helper PRIVATE include)
    target_link_libraries(test_filesystem_helper rosbag2_storage)
    ament_target_dependencies(test_filesystem_helper rosbag2_test_common)
  endif()
endif()

ament_package(
  CONFIG_EXTRAS cmake/rosbag2_storage_register_storage_plugin.cmake)

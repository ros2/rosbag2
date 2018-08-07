cmake_minimum_required(VERSION 3.5)
project(rosbag2_storage_default_plugins)

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
find_package(sqlite3_vendor REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(pluginlib REQUIRED)

find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.hpp
  src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.cpp
  src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp
  src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs SQLite3 rosbag2_storage pluginlib)
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

if(BUILD_TESTING)
  # TODO(Martin-Idel-SI): Add copyright linter or use ament_lint_auto() once available
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(sqlite_storage_test
    test/rosbag2_storage_default_plugins/sqlite/sqlite_storage_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET sqlite_storage_test)
    target_link_libraries(sqlite_storage_test ${PROJECT_NAME})
  endif()

  ament_add_gmock(sqlite_storage_integration_test
    test/rosbag2_storage_default_plugins/sqlite/sqlite_storage_integration_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET sqlite_storage_integration_test)
    target_link_libraries(sqlite_storage_integration_test ${PROJECT_NAME})
  endif()
endif()

ament_package()

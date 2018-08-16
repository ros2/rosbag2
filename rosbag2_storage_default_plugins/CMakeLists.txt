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
find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.cpp
  src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp
  src/rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rosbag2_storage
  rcutils
  SQLite3
  pluginlib)

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
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(sqlite_storage_integration_test
    test/rosbag2_storage_default_plugins/sqlite/sqlite_storage_integration_test.cpp
    src/rosbag2_storage_default_plugins/sqlite/sqlite_storage.cpp
    src/rosbag2_storage_default_plugins/sqlite/sqlite_wrapper.cpp
    src/rosbag2_storage_default_plugins/sqlite/sqlite_statement_wrapper.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET sqlite_storage_integration_test)
    ament_target_dependencies(sqlite_storage_integration_test
      rosbag2_storage
      rcutils
      SQLite3
      pluginlib)
  endif()
endif()

ament_package()

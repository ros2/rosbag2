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

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
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
  rcpputils
  rcutils
  SQLite3
  pluginlib)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ROSBAG2_STORAGE_DEFAULT_PLUGINS_BUILDING_DLL)

pluginlib_export_plugin_description_file(rosbag2_storage plugin_description.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosbag2_storage rcpputils rcutils sqlite3_vendor SQLite3)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosbag2_test_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(TEST_LINK_LIBRARIES
    ${PROJECT_NAME}
    ${rosbag2_storage_LIBRARIES}
    ${rcutils_LIBRARIES}
    ${SQLite3_LIBRARIES}
    ${pluginlib_LIBRARIES}
  )

  ament_add_gmock(test_sqlite_wrapper
    test/rosbag2_storage_default_plugins/sqlite/test_sqlite_wrapper.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_sqlite_wrapper)
    target_link_libraries(test_sqlite_wrapper ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(test_sqlite_wrapper rosbag2_test_common)
  endif()

  ament_add_gmock(test_sqlite_storage
    test/rosbag2_storage_default_plugins/sqlite/test_sqlite_storage.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_sqlite_storage)
    target_link_libraries(test_sqlite_storage ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(test_sqlite_storage rosbag2_test_common)
  endif()
endif()

ament_package()

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
find_package(std_msgs REQUIRED)
find_package(sqlite3_vendor REQUIRED)

find_package(SQLite3 REQUIRED)  # provided by sqlite3_vendor

add_library(librosbag
  include/rosbag2/rosbag2.hpp
  src/rosbag2/rosbag2.cpp
  src/rosbag2/storage/readable_storage.hpp
  src/rosbag2/storage/writable_storage.hpp
  src/rosbag2/storage/storage_factory.hpp
  src/rosbag2/storage/storage_factory.cpp
  src/rosbag2/storage/sqlite/sqlite_wrapper.hpp
  src/rosbag2/storage/sqlite/sqlite_wrapper.cpp
  src/rosbag2/storage/sqlite/sqlite_storage.hpp
  src/rosbag2/storage/sqlite/sqlite_storage.cpp)
ament_target_dependencies(librosbag rclcpp std_msgs SQLite3)
target_link_libraries(librosbag)
target_include_directories(librosbag
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_record src/rosbag2/main_record.cpp)
target_link_libraries(${PROJECT_NAME}_record librosbag)

add_executable(${PROJECT_NAME}_play src/rosbag2/main_play.cpp)
target_link_libraries(${PROJECT_NAME}_play librosbag)

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

  ament_add_gmock(sqlite_storage_test test/rosbag2/storage/sqlite_storage_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET sqlite_storage_test)
    target_link_libraries(sqlite_storage_test librosbag)
  endif()

  ament_add_gmock(storage_factory_test test/rosbag2/storage/storage_factory_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET storage_factory_test)
    target_link_libraries(storage_factory_test librosbag)
  endif()

  ament_add_gmock(rosbag2_write_integration_test test/rosbag2/rosbag2_write_integration_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_write_integration_test)
    target_link_libraries(rosbag2_write_integration_test SQlite3 librosbag)
  endif()

  ament_add_gmock(rosbag2_read_integration_test test/rosbag2/rosbag2_read_integration_test.cpp
    test/rosbag2/rosbag2_test_fixture.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_read_integration_test)
    target_link_libraries(rosbag2_read_integration_test SQlite3 librosbag)
  endif()
endif()

ament_package()

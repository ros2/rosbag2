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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_library(librosbag
  src/rosbag2/rosbag2.cpp
  src/rosbag2/storage/storage.hpp
  src/rosbag2/storage/sqlite/sqlite.hpp
  src/rosbag2/storage/sqlite/sqlite.cpp
  src/rosbag2/storage/sqlite/sqlite_storage.hpp
  src/rosbag2/storage/sqlite/sqlite_storage.cpp)
ament_target_dependencies(librosbag rclcpp std_msgs)
target_link_libraries(librosbag sqlite3)
target_include_directories(librosbag
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME} src/rosbag2/main.cpp)
target_link_libraries(${PROJECT_NAME} librosbag)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(sqlite_storage_test test/rosbag2/storage/sqlite_storage_test.cpp)
  if(TARGET sqlite_storage_test)
    target_link_libraries(sqlite_storage_test sqlite3 librosbag)
  endif()

endif()

ament_package()

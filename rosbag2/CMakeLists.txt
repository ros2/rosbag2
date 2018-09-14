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
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

add_library(
  rosbag2
  SHARED
  src/rosbag2/sequential_reader.cpp
  src/rosbag2/typesupport_helpers.cpp
  src/rosbag2/writer.cpp)

ament_target_dependencies(rosbag2
  ament_index_cpp
  Poco
  rcutils
  rosbag2_storage
  rosidl_generator_cpp
)

target_include_directories(rosbag2
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosbag2 PRIVATE "ROSBAG2_BUILDING_DLL")

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS rosbag2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)
ament_export_libraries(rosbag2)
ament_export_dependencies(rosbag2_storage)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(rosbag2_typesupport_helpers_test
    test/rosbag2/rosbag2_typesupport_helpers_test.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET rosbag2_typesupport_helpers_test)
    target_link_libraries(rosbag2_typesupport_helpers_test rosbag2)
  endif()
endif()

ament_package()

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

add_library(${PROJECT_NAME} SHARED
  src/rosbag2/sequential_reader.cpp
  src/rosbag2/typesupport_helpers.cpp
  src/rosbag2/writer.cpp
  src/rosbag2/writer_impl.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  Poco
  rcutils
  rosbag2_storage
  rosidl_generator_cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_BUILDING_DLL")

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

  ament_add_gmock(test_writer
    test/rosbag2/test_writer.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_writer)
    target_link_libraries(test_writer rosbag2)
  endif()
endif()

ament_package()

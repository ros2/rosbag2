cmake_minimum_required(VERSION 3.5)
project(rosbag2_cpp)

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

option(DISABLE_SANITIZERS "disables the use of gcc saniztizers")
# Only enable sanitizers on x64 architectures
# https://github.com/google/sanitizers/issues/794
if(NOT ${CMAKE_SIZEOF_VOID_P} EQUAL 8)
  set(DISABLE_SANITIZERS ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(poco_vendor)
find_package(Poco COMPONENTS Foundation)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2_cpp/converter.cpp
  src/rosbag2_cpp/info.cpp
  src/rosbag2_cpp/reader.cpp
  src/rosbag2_cpp/readers/sequential_reader.cpp
  src/rosbag2_cpp/serialization_format_converter_factory.cpp
  src/rosbag2_cpp/types/introspection_message.cpp
  src/rosbag2_cpp/typesupport_helpers.cpp
  src/rosbag2_cpp/types/introspection_message.cpp
  src/rosbag2_cpp/writer.cpp
  src/rosbag2_cpp/writers/sequential_writer.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  pluginlib
  Poco
  rcpputils
  rcutils
  rosbag2_storage
  rosidl_generator_cpp
  rosidl_runtime_c
  rosidl_runtime_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_typesupport_cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_CPP_BUILDING_DLL")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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
ament_export_dependencies(pluginlib rosbag2_storage rosidl_typesupport_introspection_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(test_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_library(
    converter_test_plugins
    SHARED
    test/rosbag2_cpp/serializer_test_plugin.cpp
    test/rosbag2_cpp/converter_test_plugin.cpp)
  target_link_libraries(converter_test_plugins ${PROJECT_NAME})
  install(
    TARGETS converter_test_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  pluginlib_export_plugin_description_file(rosbag2_cpp test/rosbag2_cpp/converter_test_plugin.xml)

  ament_add_gmock(test_converter_factory
    test/rosbag2_cpp/test_converter_factory.cpp)
  if(TARGET test_converter_factory)
    target_include_directories(test_converter_factory PRIVATE include)
    target_link_libraries(test_converter_factory ${PROJECT_NAME})
  endif()

  ament_add_gmock(test_typesupport_helpers
    test/rosbag2_cpp/test_typesupport_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_typesupport_helpers)
    target_link_libraries(test_typesupport_helpers ${PROJECT_NAME})
  endif()

  ament_add_gmock(test_info
    test/rosbag2_cpp/test_info.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_info)
    target_link_libraries(test_info ${PROJECT_NAME})
    ament_target_dependencies(test_info rosbag2_test_common)
  endif()

  ament_add_gmock(test_sequential_reader
    test/rosbag2_cpp/test_sequential_reader.cpp)
  if(TARGET test_sequential_reader)
    target_link_libraries(test_sequential_reader ${PROJECT_NAME})
  endif()

  ament_add_gmock(test_storage_without_metadata_file
    test/rosbag2_cpp/test_storage_without_metadata_file.cpp)
  if(TARGET test_storage_without_metadata_file)
    target_link_libraries(test_storage_without_metadata_file ${PROJECT_NAME})
  endif()

  # If compiling with gcc, run this test with sanitizers enabled
  ament_add_gmock(test_ros2_message
    test/rosbag2_cpp/types/test_ros2_message.cpp
    src/rosbag2_cpp/typesupport_helpers.cpp
    src/rosbag2_cpp/types/introspection_message.cpp)
  if(TARGET test_ros2_message)
    target_compile_definitions(test_ros2_message PRIVATE "ROSBAG2_CPP_BUILDING_DLL")
    if(NOT DISABLE_SANITIZERS)
      target_compile_options(test_ros2_message PUBLIC $<$<CXX_COMPILER_ID:GNU>:-fsanitize=leak>)
      target_link_libraries(test_ros2_message $<$<CXX_COMPILER_ID:GNU>:-fsanitize=leak>)
    endif()
    target_include_directories(test_ros2_message
      PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(test_ros2_message
      ament_index_cpp
      Poco
      rcutils
      rosbag2_storage
      rosidl_generator_cpp
      rosidl_typesupport_introspection_cpp
      rosidl_typesupport_cpp
      test_msgs)
  endif()

  ament_add_gmock(test_sequential_writer
    test/rosbag2_cpp/test_sequential_writer.cpp)
  if(TARGET test_sequential_writer)
    target_link_libraries(test_sequential_writer ${PROJECT_NAME})
  endif()

  ament_add_gmock(test_multifile_reader
    test/rosbag2_cpp/test_multifile_reader.cpp)
  if(TARGET test_multifile_reader)
    target_link_libraries(test_multifile_reader ${PROJECT_NAME})
  endif()
endif()

ament_package()

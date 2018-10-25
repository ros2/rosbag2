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
find_package(pluginlib REQUIRED)
find_package(poco_vendor)
find_package(Poco COMPONENTS Foundation)
find_package(rcutils REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/rosbag2/sequential_reader.cpp
  src/rosbag2/format_converter_factory_impl.cpp
  src/rosbag2/typesupport_helpers.cpp
  src/rosbag2/writer.cpp)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  pluginlib
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
ament_export_dependencies(rosbag2_storage)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_typesupport_helpers
    test/rosbag2/test_typesupport_helpers.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(TARGET test_typesupport_helpers)
    target_link_libraries(test_typesupport_helpers rosbag2)
  endif()

  add_library(
    converter_test_plugin
    SHARED
    test/rosbag2/converter_test_plugin.cpp)
  target_link_libraries(converter_test_plugin rosbag2)
  install(
    TARGETS converter_test_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  pluginlib_export_plugin_description_file(rosbag2 test/rosbag2/converter_test_plugin.xml)

  ament_add_gmock(test_converter_factory
    test/rosbag2/test_converter_factory.cpp)
  if(TARGET test_converter_factory)
    target_include_directories(test_converter_factory PRIVATE include)
    target_link_libraries(test_converter_factory rosbag2)
  endif()
endif()

ament_package()

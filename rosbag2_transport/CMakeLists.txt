cmake_minimum_required(VERSION 3.5)
project(rosbag2_transport)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include(cmake/configure_python.cmake)
ament_python_install_package(${PROJECT_NAME})
set(python_extension_files src/rosbag2_transport/_rosbag2_transport.c)
set_source_files_properties(${python_extension_files} PROPERTIES LANGUAGE C)
add_library(
  rosbag2_transport_py
  SHARED
  ${python_extension_files})
configure_python_c_extension_library(rosbag2_transport_py)

add_library(rosbag2_transport src/rosbag2_transport.cpp)
target_include_directories(rosbag2_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  rosbag2_transport
  "rclcpp"
  "rosbag2_storage"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rosbag2_transport PRIVATE "ROSBAG2_TRANSPORT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rosbag2_transport
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_interfaces(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  rosbag2_transport
)

ament_package()

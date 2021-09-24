cmake_minimum_required(VERSION 3.8)
project(rosbag2_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetRate.srv"
  "srv/IsPaused.srv"
  "srv/Pause.srv"
  "srv/PlayNext.srv"
  "srv/Resume.srv"
  "srv/Seek.srv"
  "srv/SetRate.srv"
  "srv/Snapshot.srv"
  "srv/TogglePaused.srv"
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

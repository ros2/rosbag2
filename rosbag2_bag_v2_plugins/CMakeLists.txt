cmake_minimum_required(VERSION 3.5)
project(rosbag2_bag_v2_plugins)

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

if(UNIX)
  find_package(ament_cmake REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rosbag REQUIRED)
  find_package(std_msgs REQUIRED)

  add_library(${PROJECT_NAME} SHARED src/rosbag2_bag_v2_plugins/storage/rosbag1_storage.cpp)

  ament_target_dependencies(${PROJECT_NAME}
    rosbag2_storage
    rosbag
    rcutils
    pluginlib)

  target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    ROSBAG1_DEFAULT_PLUGINS_BUILDING_DLL)

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
  ament_export_dependencies(rosbag2_storage)
  ament_export_dependencies(rosbag2)

  include_directories(${rosbag_INCLUDE_DIRS})
  add_executable(example_rosbag_v2 src/example_rosbag_v2.cpp)
  target_link_libraries(example_rosbag_v2 ${rosbag_LIBRARIES})

  if(BUILD_TESTING)
    find_package(ament_cmake_gmock REQUIRED)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    set(TEST_LINK_LIBRARIES
      ${PROJECT_NAME}
      ${rosbag2_storage_LIBRARIES}
      ${rcutils_LIBRARIES}
      ${pluginlib_LIBRARIES}
    )
  endif()

  ament_package()
endif()

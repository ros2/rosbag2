cmake_minimum_required(VERSION 3.8)
project(rosbag2_keyboard_handler)

# find dependencies
find_package(ament_cmake REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

set(SOURCES
    src/keyboard_handler_base.cpp
    src/keyboard_handler_unix_impl.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wthread-safety)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KEYBOARD_HANDLER_BUILDING_LIBRARY")

install(
    DIRECTORY include/
    DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  find_package(ament_cmake_gmock REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(keyboard_handler_test_sources
      test/keyboard_handler_tests.cpp)

  ament_add_gmock(test_keyboard_handler ${keyboard_handler_test_sources})
  target_include_directories(test_keyboard_handler PUBLIC include)
  target_link_libraries(test_keyboard_handler ${PROJECT_NAME})
endif()

ament_package()
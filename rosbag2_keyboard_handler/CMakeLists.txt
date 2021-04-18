cmake_minimum_required(VERSION 3.8)
project(rosbag2_keyboard_handler)

# find dependencies
find_package(ament_cmake REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
add_compile_options(-Wall)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wthread-safety)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

set(SOURCES
    include/rosbag2_keyboard_handler/visibility_control.hpp
    include/rosbag2_keyboard_handler/KeyboardHandlerBase.hpp
    include/rosbag2_keyboard_handler/IKeyboardHandler.hpp
    include/rosbag2_keyboard_handler/KeyboardHandler.hpp
    include/rosbag2_keyboard_handler/KeyboardHandlerUnixImpl.hpp
    src/IKeyboardHandler.cpp
    src/KeyboardHandlerUnixImpl.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    DIRECTORY include/
    DESTINATION include)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest)
  find_package(ament_cmake_gmock REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(keyboard_handler_test_sources
      test/FakePlayer.hpp
      test/FakeRecorder.hpp
      test/KeyboardHandlerTests.cpp)

  ament_add_gmock(test_keyboard_handler ${keyboard_handler_test_sources})
  target_include_directories(test_keyboard_handler PUBLIC include)
  target_link_libraries(test_keyboard_handler ${PROJECT_NAME})
endif()

ament_package()
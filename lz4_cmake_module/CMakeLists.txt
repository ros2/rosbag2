cmake_minimum_required(VERSION 3.20)
project(lz4_cmake_module)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)

  # Find module is intentionally mixed case, so tell linter to ignore that
  ament_lint_cmake("--filter=-convention/filename,-package/stdargs")
  ament_copyright()
endif()

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

ament_package(
  CONFIG_EXTRAS "lz4_cmake_module-extras.cmake"
)

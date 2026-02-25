cmake_minimum_required(VERSION 3.20)
project(mcap_vendor LANGUAGES C CXX)

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(lz4_cmake_module REQUIRED)
find_package(lz4 REQUIRED)
find_package(zstd_cmake_module REQUIRED)
find_package(zstd REQUIRED)

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

## Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if (MSVC)
  add_compile_options(/W4 /WX
    /wd4251 # suppress warning about having template instances (such as std::string) as public class members
  )
  # suppress error disallowing `fopen` in favor of `fopen_s`
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

## Define vendor macro
macro(build_mcap_vendor)
  include(FetchContent)
  fetchcontent_declare(mcap
    URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.1.3.tar.gz
    URL_HASH SHA1=591ece619ebff5dfe5234d91c4ea8bb30053c872 # v2.1.3
  )
  fetchcontent_makeavailable(mcap)

  add_library(mcap SHARED
    src/main.cpp
  )

  set(_mcap_include_dir ${mcap_SOURCE_DIR}/cpp/mcap/include)
  file(GLOB _mcap_installed_headers ${_mcap_include_dir}/mcap/*.hpp)

  target_include_directories(mcap PUBLIC
    "$<BUILD_INTERFACE:${_mcap_include_dir}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )
  target_link_libraries(mcap LZ4::lz4 zstd::zstd)

  install(
    FILES ${_mcap_installed_headers}
    DESTINATION include/${PROJECT_NAME}/mcap
  )

  install(
    TARGETS mcap
    EXPORT mcap
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endmacro()

## Call vendor macro
build_mcap_vendor()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(mcap HAS_LIBRARY_TARGET)
ament_export_dependencies(lz4_cmake_module lz4 zstd_cmake_module zstd)

## Package
ament_package()

cmake_minimum_required(VERSION 3.5)
project(sqlite3_vendor)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(SQLite3)

if (NOT SQLite3_FOUND)
  include(ExternalProject)
  ExternalProject_Add(sqlite3-3.24.0
    PREFIX sqlite3-3.24.0
    URL https://www.sqlite.org/2018/sqlite-amalgamation-3240000.zip
    URL_MD5 4ea1e0c6e7e82cb0490d4753d6878698
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/sqlite3_install
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/sqlite3_install"
    PATCH_COMMAND
    ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/sqlite3_cmakelists.txt" <SOURCE_DIR>/CMakeLists.txt
    )

  # The external project will install to the build folder, but we'll install that on make install.
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/sqlite3_install/ DESTINATION ${CMAKE_INSTALL_PREFIX})
else ()
  message(STATUS "Found SQLite3. Using SQLite3 from system.")
endif ()

configure_file(sqlite3_vendorConfig.cmake.in "${PROJECT_BINARY_DIR}/sqlite3_vendorConfig.cmake" @ONLY)
configure_file(sqlite3_vendorConfig-version.cmake.in "${PROJECT_BINARY_DIR}/sqlite3_vendorConfig-version.cmake" @ONLY)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

install(FILES
  "${PROJECT_BINARY_DIR}/sqlite3_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/sqlite3_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
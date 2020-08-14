cmake_minimum_required(VERSION 3.5)
project(rosbag2_py)

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
find_package(ament_cmake_ros REQUIRED)
find_package(pybind11 REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_reader SHARED
  src/rosbag2_py/_reader.cpp
)
ament_target_dependencies(_reader PUBLIC
  "rosbag2_compression"
  "rosbag2_cpp"
)

pybind11_add_module(_storage SHARED
  src/rosbag2_py/_storage.cpp
)
ament_target_dependencies(_storage PUBLIC
  "rosbag2_cpp"
  "rosbag2_storage"
)

pybind11_add_module(_writer SHARED
  src/rosbag2_py/_writer.cpp
)
ament_target_dependencies(_writer PUBLIC
  "rosbag2_compression"
  "rosbag2_cpp"
)

# Install cython modules as sub-modules of the project
install(
  TARGETS
    _reader
    _storage
    _writer
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING AND NOT (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)

  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()
  set(pythonpath "${CMAKE_CURRENT_BINARY_DIR}/rosbag2_py;${CMAKE_CURRENT_SOURCE_DIR}")
  if(NOT WIN32)
    string(REPLACE ";" ":" pythonpath "${pythonpath}")
  endif()
  ament_add_pytest_test(test_sequential_reader_py "test/test_sequential_reader.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "PYTHONPATH=${pythonpath}"
    APPEND_LIBRARY_DIRS "${_append_library_dirs}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources"
  )
  ament_add_pytest_test(test_sequential_writer_py "test/test_sequential_writer.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "PYTHONPATH=${pythonpath}"
    APPEND_LIBRARY_DIRS "${_append_library_dirs}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources"
  )
endif()

ament_package()

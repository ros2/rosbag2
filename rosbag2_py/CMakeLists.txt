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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)

# Find python before pybind11
find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)
find_package(Python3 REQUIRED Interpreter Development)
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # Set the python debug interpreter.
  # We need to set *both* PYTHON_EXECUTABLE (for use in this CMake file),
  # as well as Python3_EXECUTABLE (which is what pybind11 uses).
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_reader SHARED
  src/rosbag2_py/_reader.cpp
)
ament_target_dependencies(_reader PUBLIC
  "rosbag2_compression"
  "rosbag2_cpp"
  "rosbag2_storage"
)

pybind11_add_module(_storage SHARED
  src/rosbag2_py/_storage.cpp
  src/rosbag2_py/format_bag_metadata.cpp
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
  "rosbag2_storage"
)

pybind11_add_module(_info SHARED
  src/rosbag2_py/_info.cpp
)
ament_target_dependencies(_info PUBLIC
  "rosbag2_cpp"
  "rosbag2_storage"
)

pybind11_add_module(_transport SHARED
  src/rosbag2_py/_transport.cpp
)
ament_target_dependencies(_transport PUBLIC
  "rosbag2_compression"
  "rosbag2_cpp"
  "rosbag2_storage"
  "rosbag2_transport"
)

pybind11_add_module(_reindexer SHARED
  src/rosbag2_py/_reindexer.cpp
)
ament_target_dependencies(_reindexer PUBLIC
  "rosbag2_cpp"
  "rosbag2_storage"
)

# Install cython modules as sub-modules of the project
install(
  TARGETS
    _reader
    _storage
    _writer
    _info
    _transport
    _reindexer
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)

  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()
  set(append_env_vars "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  set(set_env_vars "ROSBAG2_PY_TEST_RESOURCES_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # Work around a rtti bug when using class_loader from a
    # python extension built with libc++.
    set(set_env_vars "${set_env_vars} ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL=True")
  endif()

  # message(FATAL_ERROR "HELP I AM THE PATH ${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_sequential_reader_py "test/test_sequential_reader.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
  ament_add_pytest_test(test_sequential_reader_multiple_files_py "test/test_sequential_reader_multiple_files.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
  ament_add_pytest_test(test_sequential_writer_py
    "test/test_sequential_writer.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
  ament_add_pytest_test(test_transport_py
    "test/test_transport.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
  ament_add_pytest_test(test_reindexer_py
    "test/test_reindexer.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
  ament_add_pytest_test(test_convert_py
    "test/test_convert.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
    APPEND_ENV "${append_env_vars}"
    ENV "${set_env_vars}"
  )
endif()

ament_package()

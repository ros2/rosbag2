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
find_package(rosbag2_cpp REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_rosbag2_py SHARED
  src/rosbag2_py/_rosbag2_py.cpp
)
# TODO(jacobperron): Consider modifying ament_python_install_module to accept targets
# ament_python_install_module(_rosbag2_py
#   DESTINATION_SUFFIX "rosbag2_py"
# )
# target_link_libraries(_rosbag2_py

ament_target_dependencies(_rosbag2_py PUBLIC
  "rosbag2_cpp"
)

# Install cython modules as sub-modules of the project
install(
  TARGETS _rosbag2_py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # TODO(jacobperron): Add pytests
endif()

ament_package()

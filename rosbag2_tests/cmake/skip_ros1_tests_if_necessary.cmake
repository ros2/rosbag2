# Copyright 2018, Bosch Software Innovations GmbH.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Macro to set variable SKIP_ROS1_TESTS if ros1 is not installed
# Some end to end tests rely on the plugin to read rosbags from ROS 1.
# Those tests should be marked with SKIP_ROS1_TESTS and will be skipped if the
# plugin is not available

macro(skip_ros1_tests_if_necessary)
  find_package(PkgConfig)
  if(PKG_CONFIG_FOUND)
    find_package(ros1_bridge REQUIRED)
    include(${ros1_bridge_DIR}/find_ros1_interface_packages.cmake)
    include(${ros1_bridge_DIR}/find_ros1_package.cmake)
    find_ros1_package(roscpp)
    if(NOT ros1_roscpp_FOUND)
      set(SKIP_ROS1_TESTS "SKIP_TEST")
      message(WARNING
        "Skipping build of tests for rosbag_v2 plugin. ROS 1 not found")
    endif()
  else()
    set(SKIP_ROS1_TESTS "SKIP_TEST")
    message(WARNING
      "Skipping build of tests for rosbag_v2 plugin. ROS 1 not found")
  endif()
endmacro()
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#
# Create a rosbag2_transport specific gmock test for a given rmw implementation
#
# The following variables must be set before calling this function. Note that
# these are set by `call_for_each_rmw_implementation`, so if you call this
# within that context, the constraint is already met:
# * rmw_implementation: The package name of the RMW implementation
# * target_suffix: Either a string derived from the RMW implementation or empty
#   if there is only one RMW implementation
#
# :param target_base: the base name of the test to create, it will be suffixed
# :type target_base: string
# :param SKIP_TEST: if set, do not actually run the test at test time
# :type GENERATE_DEFAULT: option
# :param LINK_LIBS: libraries to link to the test executable
# :type LINK_LIBS: list of strings
# :param AMENT_DEPS: ament dependencies to declare for the test executable
# :type AMENT_DEPS: list of strings
# :param INCLUDE_DIRS: extra include directories to use for building the test
# :type INCLUDE_DIRS: list of strings
#
function(rosbag2_transport_add_gmock target_base)
  cmake_parse_arguments(ARG
    "SKIP_TEST"
    ""
    "LINK_LIBS;AMENT_DEPS;INCLUDE_DIRS"
    ${ARGN})
  if(NOT ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "rosbag2_transport_add_gmock() must be invoked with "
      "at least one source file")
  endif()
  if(${ARG_SKIP_TEST})
    set(SKIP_TEST "SKIP_TEST")
  else()
    set(SKIP_TEST "")
  endif()

  set(target_name ${target_base}${target_suffix})
  set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})
  ament_add_gmock(${target_name}
    ${ARG_UNPARSED_ARGUMENTS}
    ENV ${rmw_implementation_env_var}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ${SKIP_TEST})
  if(TARGET ${target_name})
    ament_target_dependencies(${target_name} ${ARG_AMENT_DEPS})
    target_link_libraries(${target_name} ${ARG_LINK_LIBS})
    target_include_directories(${target_name} PUBLIC ${ARG_INCLUDE_DIRS})
  endif()
endfunction()

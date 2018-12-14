// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "rosbag2/typesupport_helpers.hpp"

using namespace ::testing;  // NOLINT

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_no_type) {
  EXPECT_ANY_THROW(rosbag2::extract_type_and_package("just_a_package_name"));
}

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_multiple_slashes) {
  EXPECT_ANY_THROW(rosbag2::extract_type_and_package("name/with/multiple_slashes"));
}

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_slash_at_the_start_only) {
  EXPECT_ANY_THROW(rosbag2::extract_type_and_package("/name_with_slash_at_start"));
}

TEST(TypesupportHelpersTest, throws_exception_if_filetype_has_slash_at_the_end_only) {
  EXPECT_ANY_THROW(rosbag2::extract_type_and_package("name_with_slash_at_end/"));
}

TEST(TypesupportHelpersTest, separates_into_package_and_name_for_correct_package) {
  std::string package;
  std::string name;
  std::tie(package, name) = rosbag2::extract_type_and_package("package/name");

  EXPECT_THAT(package, StrEq("package"));
  EXPECT_THAT(name, StrEq("name"));
}

TEST(TypesupportHelpersTest, throws_exception_if_library_cannot_be_found) {
  EXPECT_THROW(
    rosbag2::get_typesupport("invalid/message", "rosidl_typesupport_cpp"), std::runtime_error);
}

TEST(TypesupportHelpersTest, returns_c_type_info_for_valid_library) {
  auto string_typesupport =
    rosbag2::get_typesupport("test_msgs/Primitives", "rosidl_typesupport_cpp");

  EXPECT_THAT(std::string(string_typesupport->typesupport_identifier),
    ContainsRegex("rosidl_typesupport"));
}

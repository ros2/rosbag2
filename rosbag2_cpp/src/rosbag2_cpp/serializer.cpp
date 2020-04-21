// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rosbag2_cpp/serializer.hpp"

namespace
{
/// Default serialization format is CDR
const char * const kSerializationFormat = "cdr";
/// A string for creating introspection type support
const char * const kIntrospectionString = "rosidl_typesupport_introspection_cpp";
/// A string for creating type support
const char * const kTypeSupportString = "rosidl_typesupport_cpp";
}

namespace rosbag2_cpp
{

Serializer::Serializer()
: factory_(),
  serializer_(factory_.load_serializer(kSerializationFormat)),
  allocator_(rcutils_get_default_allocator()),
  introspection_library_(),
  type_support_library_(),
  type_support_string_(kTypeSupportString),
  introspection_string_(kIntrospectionString)
{
}

}  // namespace rosbag2_cpp

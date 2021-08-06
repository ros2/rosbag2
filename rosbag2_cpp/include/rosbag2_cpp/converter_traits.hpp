// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_CPP__CONVERTER_TRAITS_HPP_
#define ROSBAG2_CPP__CONVERTER_TRAITS_HPP_

namespace rosbag2_cpp
{

  namespace converter_interfaces {
    class SerializationFormatSerializer;
    class SerializationFormatDeserializer;
    class SerializationFormatConverter;
  }

template<typename T>
struct ConverterTraits
{};

template<>
struct ConverterTraits<converter_interfaces::SerializationFormatConverter>
{
  static constexpr const char * name =
    "rosbag2_cpp::converter_interfaces::SerializationFormatConverter";
};

template<>
struct ConverterTraits<converter_interfaces::SerializationFormatSerializer>
{
  static constexpr const char * name =
    "rosbag2_cpp::converter_interfaces::SerializationFormatSerializer";
};

template<>
struct ConverterTraits<converter_interfaces::SerializationFormatDeserializer>
{
  static constexpr const char * name =
    "rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer";
};

}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CONVERTER_TRAITS_HPP_

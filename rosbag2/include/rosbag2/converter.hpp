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

#ifndef ROSBAG2__CONVERTER_HPP_
#define ROSBAG2__CONVERTER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rosbag2/serialization_format_converter_factory.hpp"
#include "rosbag2/serialization_format_converter_factory_interface.hpp"
#include "rosbag2/serialization_format_converter_interface.hpp"
#include "rosbag2/types.hpp"
#include "rosbag2/visibility_control.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2
{
class ROSBAG2_PUBLIC Converter
{
public:
  explicit
  Converter(
    const std::string & input_format,
    const std::string & output_format,
    const std::vector<TopicWithType> & topics_and_types,
    std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<SerializationFormatConverterFactory>());

  ~Converter();

  /**
   * Converts the given SerializedBagMessage into the output format of the converter. The
   * serialization format of the input message must be identical to the input format of the
   * converter.
   *
   * \param message Message to convert
   * \returns Converted message
   */
  std::shared_ptr<SerializedBagMessage>
  convert(std::shared_ptr<const SerializedBagMessage> message);

private:
  std::shared_ptr<SerializationFormatConverterFactoryInterface> converter_factory_;
  std::unique_ptr<SerializationFormatConverterInterface> input_converter_;
  std::unique_ptr<SerializationFormatConverterInterface> output_converter_;
  std::map<std::string, std::string> topics_and_types_;
};

}  // namespace rosbag2

#endif  // ROSBAG2__CONVERTER_HPP_

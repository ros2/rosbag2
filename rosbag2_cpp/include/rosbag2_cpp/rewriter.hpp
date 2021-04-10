// Copyright 2018, Bosch Software Innovations GmbH.
// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
// Copyright 2021 Firefly Automatix, Inc.
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

#ifndef ROSBAG2_CPP__REWRITER_HPP_
#define ROSBAG2_CPP__REWRITER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{

/**
 * The Rewriter will open a bagfile and rewrite it into another bagfile.
 * This can be used to "reshape" bagfiles, or even convert storage implementation,
 * compression, or serialization format by passing in different options (storage or converter)
 */
class ROSBAG2_CPP_PUBLIC Rewriter final
{
public:
  explicit Rewriter(
    std::unique_ptr<Reader> reader_ =
    std::make_unique<Reader>(),
    std::unique_ptr<Writer> writer_ =
    std::make_unique<Writer>());

  ~Rewriter();

  /**
   * Opens an existing bagfile, and rewrites it into another bagfile
   * with new settings to a new location.
   *
   * \param input_storage_options Storage options for the input bagfile
   * \param input_converter_options Converter options for the input bagfile
   * \param output_storage_options Storage options for the output bagfile
   * \param output_converter_options Converter options for the output bagfile
   **/
  void rewrite(
    rosbag2_storage::StorageOptions input_storage_options,
    ConverterOptions input_converter_options,
    rosbag2_storage::StorageOptions output_storage_options,
    ConverterOptions output_converter_options
  );

  /**
   * Opens a a vector of existing bagfiles, and rewrites it into another bagfile
   * with new settings to a new location.
   *
   * \note the API consumer is responsible for sorting the input_storage_options according to desired order.
   *
   * \param input_storage_options Storage options for the input bagfile.
   * \param input_converter_options Converter options for the input bagfile
   * \param output_storage_options Storage options for the output bagfile
   * \param output_converter_options Converter options for the output bagfile
   **/
  void rewrite_many(
    std::vector<rosbag2_storage::StorageOptions> input_storage_options,
    ConverterOptions input_converter_options,
    rosbag2_storage::StorageOptions output_storage_options,
    ConverterOptions output_converter_options
  );

private:
  std::unique_ptr<Reader> reader_;
  std::unique_ptr<Writer> writer_;
};

}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__REWRITER_HPP_

// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2__COMPRESSION_INTERFACES__BASE_COMPRESSOR_INTERFACE_HPP_
#define ROSBAG2__COMPRESSION_INTERFACES__BASE_COMPRESSOR_INTERFACE_HPP_

#include <memory>
#include <string>

#include "rosbag2/visibility_control.hpp"

/**
 * An interface for developers adding a new compression algorithm to rosbag2. These functions
 * must be implemented to that a writer can properly compress a file or bag message.
 * A corresponding decompressor must also be implemented.
 *
 * Example file compression usage:
 *
 * MyCompressor my_compressor();
 * std::string current_uri = storage.get_relative_path();
 * std::string compressed_path_uri = my_compressor.compress_uri(current_uri);
 * relative_file_paths.push_back(compressed_path_uri);
 *
 * Example message compression usage:
 *
 * MyCompressor my_compressor();
 * std::shared_ptr<SerializedBagMessage> bag_message = std::make_shared<SerializedBagMessage>();
 * ...fill message
 * std::shared_ptr<SerializedBagMessage> compressed_message =
 *   my_compressor.compress_serialized_bag_message(bag_message);
 */
namespace rosbag2
{

class ROSBAG2_PUBLIC BaseCompressorInterface
{
public:
  virtual ~BaseCompressorInterface() {}

  /**
   * Compress a file on disk.
   *
   * \param uri Input file to compress with file extension.
   * \return The relative path to the compressed file with the compressed extension.
   */
  virtual std::string compress_uri(const std::string & uri) = 0;

  /**
   * Compress serialized_data in a serialized bag message.
   *
   * \param bag_message A serialized bag message.
   * \return A shared pointer to the bag message with compressed serialized_data.
   */
  virtual std::shared_ptr<SerializedBagMessage> compress_serialized_bag_message(
    std::shared_ptr<SerializedBagMessage> bag_message) = 0;

  /**
   * Get the identifier of the compression algorithm. This is appended to the relative file path.
   */
  virtual std::string get_compression_identifier() const = 0;
};

}  // namespace rosbag2

#endif  // ROSBAG2__COMPRESSION_INTERFACES__BASE_COMPRESSOR_INTERFACE_HPP_

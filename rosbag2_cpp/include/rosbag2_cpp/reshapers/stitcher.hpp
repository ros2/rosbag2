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

#ifndef ROSBAG2_CPP__RESHAPERS__STITCHER_HPP_
#define ROSBAG2_CPP__RESHAPERS__STITCHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_cpp
{
namespace reader_interfaces
{
class BaseReaderInterface;
}  // namespace reader_interfaces
namespace writer_interfaces
{
class BaseWriterInterface;
}  // namespace writer_interfaces

class ROSBAG2_CPP_PUBLIC Stitcher final
{
public:
  explicit Stitcher(
    std::unique_ptr<reader_interfaces::BaseReaderInterface> reader_impl =
    std::make_unique<readers::SequentialReader>(),
    std::unique_ptr<writer_interfaces::BaseWriterInterface> writer_impl =
    std::make_unique<writers::SequentialWriter>());
  
  ~Stitcher();

  /**
   * Opens the list of URIs and prepares it for stitching.
   * Each URI in this vector must be a bagfile that exists.
   * This must be called before any other function is used.
   * 
   * \note This will step through the directory with the default storage options
   * * using sqlite3 storage backend
   * * using no converter options, storing messages with the incoming serialization format
   * \sa rmw_get_serialization_format.
   * For specifications, please see \sa open, which let's you specify
   * more storage and converter options.
   * 
   * \param storage_uris A vector of URI of the storage to open.
   * \param output_uri The uri of the storage to write to.
   **/
  void open(const std::vector<std::string> & storage_uris, const std::string & output_uri);

  /**
   * Ask whether the stitcher has at least one more URI to stitch.
   * 
   * \return true if there remains at least one more unstitched bag
   * \throws runtime_error if Stitcher is not open.
   */
  bool has_next();

  /**
   * Process next message from list of storage. Will throw if no more storage URI
   * to stitch.
   * 
   * Expected usage:
   * if (stitcher.has_next()) {stitcher.stitch_next();}
   */
  void stitch_next();

  /**
   * Set filters to adhere to during stitching.
   *
   * \param storage_filter Filter to apply to stitching
   * \throws runtime_error if the Stitcher is not open.
   */
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter);

  /**
   * Reset all filters for stitching.
   */
  void reset_filter();

  reader_interfaces::BaseReaderInterface & get_reader_implementation_handle() const
  {
    return *reader_impl_;
  }

  writer_interfaces::BaseWriterInterface & get_writer_implementation_handle() const
  {
    return *writer_impl_;
  }

private:
  std::unique_ptr<reader_interfaces::BaseReaderInterface> reader_impl_;
  std::unique_ptr<writer_interfaces::BaseWriterInterface> writer_impl_;
};

}  // namespace rosbag2_cpp

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // ROSBAG2_CPP__RESHAPERS__STITCHER_HPP_

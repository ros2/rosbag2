// Copyright 2025 Apex.AI, Inc.
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

#ifndef ROSBAG2_TRANSPORT__READERS_MANAGER_HPP_
#define ROSBAG2_TRANSPORT__READERS_MANAGER_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "rcutils/time.h"
#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_transport/visibility_control.hpp"

#ifdef _WIN32
#  pragma warning(push)
// Suppress warning "rosbag2_transport::ReadersManager::pimpl_': class 'std::unique_ptr>'
// needs to have dll-interface to be used by clients of class 'rosbag2_transport::ReadersManager'"
// Justification:
// 1. We never inline code in the header that actually calls methods on ReadersManagerImpl.
// 2. While the `ReadersManagerImpl` is defined in the `readers_manager_impl.hpp`
// file, we include it only in the `readers_manager.cpp` file, and it does not leak into the
// external API.
// 3. The pimpl design pattern imply that implementation details are hidden and shouldn't be
// exposed with the dll-interface.
#  pragma warning(disable:4251)
#endif


namespace rosbag2_transport
{
class ReadersManagerImpl;

/// \class ReadersManager
/// \brief This class manages multiple rosbag2_cpp::Reader instances, allowing for
/// chronological reading of messages across all readers.
/// \details It maintains a cache of the next message from each reader and provides methods to
/// retrieve the next message in chronological order, seek to a specific timestamp across all
/// readers, and apply filters to all readers. It also provides access to the earliest and
/// latest timestamps across all readers.
class ROSBAG2_TRANSPORT_PUBLIC ReadersManager
{
public:
  using reader_storage_options_pair_t =
    std::pair<std::unique_ptr<rosbag2_cpp::Reader>, rosbag2_storage::StorageOptions>;

  /// \brief Constructor which initializes the ReadersManager with multiple readers and their
  /// associated storage options.
  /// \note The readers will be opened during construction and cache will be populated with the
  /// first message from each reader (if available).
  /// \param reader_with_options Vector of pairs of unique pointer to the rosbag2_cpp::Reader class
  /// (which will be moved to the internal instance of the ReadersManager class during construction)
  /// and storage options (which will be applied to the rosbag2_cpp::reader when opening it).
  explicit ReadersManager(std::vector<reader_storage_options_pair_t> && reader_with_options);

  /// \brief Deleted default constructor and copy/move operations.
  ReadersManager() = delete;
  ReadersManager(const ReadersManager &) = delete;
  ReadersManager & operator=(const ReadersManager &) = delete;
  ReadersManager(ReadersManager &&) = delete;
  ReadersManager & operator=(ReadersManager &&) = delete;

  /// \brief Destructor which cleans up resources used by the ReadersManager.
  /// \note The readers will be closed during destruction.
  virtual ~ReadersManager();

  /// \brief Getter for the currently stored storage options
  /// \return Copy of the currently stored storage options
  [[nodiscard]] std::vector<rosbag2_storage::StorageOptions> get_all_storage_options() const;

  /// \brief Check if there are present some messages to take.
  /// \details Indicates that all readers have been exhausted.
  /// i.e. there are no more messages to take from readers.
  /// \return true if there are more messages to take, false otherwise.
  [[nodiscard]] bool has_next() const;

  /// \brief Get the next message with the earliest recv_timestamp. Updates the cache by
  /// reading from readers as necessary.
  [[nodiscard]] std::shared_ptr<rosbag2_storage::SerializedBagMessage>
  get_next_message_in_chronological_order();

  /// \brief Seek all readers to the provided timestamp.
  /// \details seek(t) will cause subsequent reads from readers to return messages that satisfy
  /// timestamp >= time t.
  /// \param timestamp The timestamp to seek to.
  void seek(const rcutils_time_point_value_t & timestamp);

  /// \brief Getter method for the earliest time among all readers.
  /// \return Returns timestamp of the first message in nanoseconds.
  [[nodiscard]] rcutils_time_point_value_t get_earliest_timestamp() const;

  /// \brief Getter method for te latest time among all readers.
  /// \return Returns timestamp of the last message in nanoseconds.
  [[nodiscard]] rcutils_time_point_value_t get_latest_timestamp() const;

  /// \brief Apply a filter to all readers.
  /// \param storage_filter The filter to apply.
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter);

  /// \brief Getter for all topics and types in all readers.
  /// \return vector of topics with topic name and type as std::string
  [[nodiscard]] std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() const;

  /// \brief Add event callbacks to all readers.
  /// \param callbacks The callbacks to add.
  void add_event_callbacks(rosbag2_cpp::bag_events::ReaderEventCallbacks & callbacks);

private:
  std::unique_ptr<ReadersManagerImpl> pimpl_;
};

}  // namespace rosbag2_transport

#ifdef _WIN32
#  pragma warning(pop)
#endif

#endif  // ROSBAG2_TRANSPORT__READERS_MANAGER_HPP_

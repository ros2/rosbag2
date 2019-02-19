// Copyright 2019, Denso ADAS Engineering Services GmbH.
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

#include "rosbag2_storage_default_plugins/sqlite/sqlite_sequential_storage.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include "rosbag2_storage/filesystem_helper.hpp"
#include "rosbag2_storage/ros_helper.hpp"

#include "../logging.hpp"

namespace rosbag2_storage_plugins
{

void SqliteSequentialStorage::open(
  const std::string & uri, rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  SqliteStorage::open(uri, io_flag);

  const bool ro = is_read_only(io_flag);
  auto metadata = ro ?
    load_metadata(uri) :
    std::unique_ptr<rosbag2_storage::BagMetadata>();

  // in write mode we add at least one serialized file
  // (it would be possible to split the data into into multiple files)
  if (!ro) {
    data_files_.push_back(
      std::make_unique<BinarySequentialFile>(database_name_ + ".extra")
    );
  } else if (metadata->relative_file_paths.size() >= 1) {
    std::for_each(metadata->relative_file_paths.begin() + 1,
      metadata->relative_file_paths.end(), [this](auto & path)
      {
        data_files_.push_back(
          std::make_unique<BinarySequentialFile>(path)
        );
      });
  }

  if (data_files_.size() == 0) {  // no serialized file
    throw std::runtime_error("Missing binary, sequential file!");
  } else {
    for (auto & data_file : data_files_) {
      data_file->file_stream.open(
        rosbag2_storage::FilesystemHelper::concat({uri, data_file->path}),
        std::fstream::binary | (ro ? std::fstream::in : std::fstream::out)
      );

      if (!data_file->file_stream) {
        throw std::runtime_error("Failed to setup storage. Failed to open extra file!");
      }
    }
  }
}

std::shared_ptr<rcutils_uint8_array_t> SqliteSequentialStorage::get_serialized_data(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  constexpr uint32_t file_index = 0;  // could be changed to split data into files

  // allocate array for message meta information
  auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(new rcutils_uint8_array_t,
      [](rcutils_uint8_array_t * msg) {
        int error = rcutils_uint8_array_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_storage_default_plugins", "Leaking memory %i", error);
        }
      });
  *serialized_data = rcutils_get_zero_initialized_uint8_array();

  auto allocator = rcutils_get_default_allocator();
  auto ret = rcutils_uint8_array_init(serialized_data.get(), sizeof(MetaInformation), &allocator);
  if (ret != RCUTILS_RET_OK) {
    throw std::runtime_error("Error allocating resources " + std::to_string(ret));
  }
  MetaInformation * data = reinterpret_cast<MetaInformation *>(serialized_data->buffer);
  serialized_data->buffer_length = sizeof(MetaInformation);

  // write actual content to file and store meta information to serialized data to return
  auto & file_handle = data_files_[file_index];
  std::lock_guard<std::mutex>(file_handle->lock);

  data->offset = static_cast<uint64_t>(file_handle->file_stream.tellg());
  data->size = static_cast<uint64_t>(message->serialized_data->buffer_length);
  data->file_index = file_index;

  file_handle->file_stream.write(reinterpret_cast<const char *>(message->serialized_data->buffer),
    message->serialized_data->buffer_length);

  return serialized_data;
}


std::shared_ptr<rcutils_uint8_array_t> SqliteSequentialStorage::read_data()
{
  std::shared_ptr<rcutils_uint8_array_t> serialized_return_data;

  MetaInformation * data;
  std::shared_ptr<rcutils_uint8_array_t> serialized_data = std::get<0>(*current_message_row_);
  if (serialized_data->buffer_length == sizeof(MetaInformation)) {
    data = reinterpret_cast<MetaInformation *>(serialized_data->buffer);

    if (!data || data->file_index >= data_files_.size()) {
      throw std::runtime_error("Invalid content for extra file!");
    }

    auto & file_handle = data_files_[data->file_index];
    std::lock_guard<std::mutex>(file_handle->lock);

    serialized_return_data = rosbag2_storage::make_empty_serialized_message(data->size);
    serialized_return_data->buffer_length = data->size;
    file_handle->file_stream.seekg(data->offset);

    file_handle->file_stream.read(reinterpret_cast<char *>(
        serialized_return_data->buffer), data->size
    );
  } else {
    throw std::runtime_error("Database entry has wrong size!");
  }

  return serialized_return_data;
}

rosbag2_storage::BagMetadata SqliteSequentialStorage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata = SqliteStorage::get_metadata();
  metadata.storage_identifier = get_storage_identifier();
  for (const auto & f : data_files_) {
    metadata.relative_file_paths.push_back(f->path);
  }

  return metadata;
}

const std::string SqliteSequentialStorage::get_storage_identifier() const
{
  return "sqlite3_sequential";
}

}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(rosbag2_storage_plugins::SqliteSequentialStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)

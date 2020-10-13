// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include "rosbag2_compression/sequential_compression_writer.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcpputils/filesystem_helper.hpp"

#include "rcutils/filesystem.h"

#include "rosbag2_compression/zstd_compressor.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/storage_options.hpp"

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "logging.hpp"

namespace rosbag2_compression
{

namespace
{
std::string format_storage_uri(const std::string & base_folder, uint64_t storage_count)
{
  // Right now `base_folder_` is always just the folder name for where to install the bagfile.
  // The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  std::stringstream storage_file_name;
  storage_file_name << rcpputils::fs::path(base_folder).filename().string() << "_" << storage_count;

  return (rcpputils::fs::path(base_folder) / storage_file_name.str()).string();
}
}  // namespace

SequentialCompressionWriter::SequentialCompressionWriter(
  const rosbag2_compression::CompressionOptions & compression_options)
: SequentialWriter(),
  compression_factory_{std::make_unique<rosbag2_compression::CompressionFactory>()},
  compression_options_{compression_options}
{}

SequentialCompressionWriter::SequentialCompressionWriter(
  const rosbag2_compression::CompressionOptions & compression_options,
  std::unique_ptr<rosbag2_compression::CompressionFactory> compression_factory,
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: SequentialWriter(std::move(storage_factory), converter_factory, std::move(metadata_io)),
  compression_factory_{std::move(compression_factory)},
  compression_options_{compression_options}
{}

SequentialCompressionWriter::~SequentialCompressionWriter()
{
  reset();
}

void SequentialCompressionWriter::init_metadata()
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  metadata_ = rosbag2_storage::BagMetadata{};
  metadata_.storage_identifier = storage_->get_storage_identifier();
  metadata_.starting_time = std::chrono::time_point<std::chrono::high_resolution_clock>{
    std::chrono::nanoseconds::max()};
  metadata_.relative_file_paths = {storage_->get_relative_file_path()};
  metadata_.compression_format = compression_options_.compression_format;
  metadata_.compression_mode =
    rosbag2_compression::compression_mode_to_string(compression_options_.compression_mode);
}

void SequentialCompressionWriter::setup_compression()
{
  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::NONE) {
    throw std::invalid_argument{
            "SequentialCompressionWriter requires a CompressionMode that is not NONE!"};
  }

  setup_compressor_threads();
}

void SequentialCompressionWriter::setup_compressor_threads()
{
  ROSBAG2_COMPRESSION_LOG_DEBUG(
    "setup_compressor_threads: Starting %lu threads",
    compression_options_.compression_threads);
  if (compression_options_.compression_threads < 1) {
    // It shouldn't be possible for this to actually be set to less than 1 at
    // this point, but we'll check it as a safety test, since if it's not at
    // least 1, nothing will ever get compressed.
    compression_options_.compression_threads = 1;
  }
  compression_is_running_ = true;
  auto compress_fn = [&]()
    {
      // Every thread needs to have its own compression context for thread safety.
      auto compressor = compression_factory_->create_compressor(
        compression_options_.compression_format);

      std::mutex mutex;
      std::unique_lock<std::mutex> lock(mutex);

      if (!compressor) {
        throw std::runtime_error{
                "Cannot compress message; Writer is not open!"};
      }

      while (compression_is_running_) {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
        std::string file;
        compressor_condition_.wait(lock);
        {
          std::lock_guard<std::mutex> queue_lock(compressor_mutex_);
          if (!compressor_message_queue_.empty()) {
            message = compressor_message_queue_.front();
            compressor_message_queue_.pop();
          } else if (!compressor_file_queue_.empty()) {
            file = compressor_file_queue_.front();
            compressor_file_queue_.pop();
          }
        }

        if (message) {
          compress_message(*compressor, message);

          {
            // Now that the message is compressed, it can be written to file using the
            // normal method.
            std::lock_guard<std::recursive_mutex> storage_lock(
              storage_mutex_);
            SequentialWriter::write(message);
          }
        } else if (!file.empty()) {
          compress_file(*compressor, file);
        }
      }
    };

  // This function needs to throw an exception if the compression format is invalid, but because
  // each thread creates its own compressor, we can't actually catch it here if one of the threads
  // fails.  Instead, we'll create a compressor that we don't actually use just so that it will
  // throw an exception if the format is invalid.
  auto compressor = compression_factory_->create_compressor(
    compression_options_.compression_format);
  if (!compressor) {
    throw std::runtime_error{
            "Cannot compress message; Writer is not open!"};
  }

  for (uint64_t i = 0; i < compression_options_.compression_threads; i++) {
    compression_threads_.emplace_back(compress_fn);
  }
}

void SequentialCompressionWriter::stop_compressor_threads()
{
  if (!compression_threads_.empty()) {
    ROSBAG2_COMPRESSION_LOG_DEBUG("Waiting for compressor threads to finish.");
    compression_is_running_ = false;
    compressor_condition_.notify_all();
    for (auto & thread : compression_threads_) {
      thread.join();
    }
    compression_threads_.clear();
  }
}

void SequentialCompressionWriter::open(
  const rosbag2_cpp::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::open(storage_options, converter_options);
  setup_compression();
}

void SequentialCompressionWriter::reset()
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  if (!base_folder_.empty()) {
    // Reset may be called before initializing the compressor (ex. bad options).
    // We compress the last file only if it hasn't been compressed earlier (ex. in split_bagfile()).
    if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE &&
      should_compress_last_file_)
    {
      try {
        storage_.reset();  // Storage must be closed before it can be compressed.
        if (!metadata_.relative_file_paths.empty())
        {
          std::string file = metadata_.relative_file_paths.back();
          compressor_file_queue_.push(file);
          compressor_condition_.notify_one();
        }
      } catch (const std::runtime_error & e) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM("Could not compress the last bag file.\n" << e.what());
      }
    }

    stop_compressor_threads();

    finalize_metadata();
    metadata_io_->write_metadata(base_folder_, metadata_);
  }

  storage_.reset();  // Necessary to ensure that the storage is destroyed before the factory
  storage_factory_.reset();
}

void SequentialCompressionWriter::create_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::create_topic(topic_with_type);
}

void SequentialCompressionWriter::remove_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::remove_topic(topic_with_type);
}

void SequentialCompressionWriter::compress_file(
  BaseCompressorInterface & compressor,
  const std::string & file)
{
  ROSBAG2_COMPRESSION_LOG_DEBUG("Compressing file: %s", file.c_str());
  const auto to_compress = rcpputils::fs::path{file};

  if (to_compress.exists() && to_compress.file_size() > 0u) {
    const auto compressed_uri = compressor.compress_uri(to_compress.string());

    {
      // After we've compressed the file, replace the name in the file list with the
      // new name.
      // It feels inefficient here to just search through the vector for the file name,
      // but it's the easiest way to handle this; both the size and the order of this vector
      // are important, since they're used to handle the generation of new files names and the
      // order of the bag files.  If we have multiple threads that could all be compressing
      // files at once, there's no guarantee in which order they'll finish.  All that means
      // that we can't remove items or change their order after they're added, and we can't
      // rely on the file we just finished compressing being in any particular place.  We can't
      // even rely on holding on to a reference to an iterator in the vector, since newly added
      // entries could have invalidated that vector.  We just have to go find our entry again.
      std::lock_guard<std::mutex> lock(compressor_mutex_);
      auto iter = std::find(
        metadata_.relative_file_paths.begin(),
        metadata_.relative_file_paths.end(),
        file);
      if (iter != metadata_.relative_file_paths.end()) {
        *iter = compressed_uri;
      } else {
        ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
          "Failed to find path to uncompressed bag: \"" << file <<
            "\"; this shouldn't happen.");
      }
    }

    if (!rcpputils::fs::remove(to_compress)) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
        "Failed to remove uncompressed bag: \"" << to_compress.string() << "\"");
    }
  } else {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
      "Removing last file: \"" << to_compress.string() <<
        "\" because it either is empty or does not exist.");
  }
}

void SequentialCompressionWriter::split_bagfile()
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  std::lock_guard<std::mutex> compressor_lock(compressor_mutex_);

  const auto storage_uri = format_storage_uri(
    base_folder_,
    metadata_.relative_file_paths.size());

  storage_ = storage_factory_->open_read_write(storage_uri, metadata_.storage_identifier);

  // If we're in FILE compression mode, push this file's name on to the queue so another
  // thread will handle compressing it.  If not, we can just carry on.
  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE) {
    std::string file = metadata_.relative_file_paths.back();
    compressor_file_queue_.push(file);
    compressor_condition_.notify_one();
  }

  if (!storage_) {
    // Add a check to make sure reset() does not compress the file again if we couldn't load the
    // storage plugin.
    should_compress_last_file_ = false;

    std::stringstream errmsg;
    errmsg << "Failed to rollover bagfile to new file: \"" << storage_uri << "\"!";
    throw std::runtime_error{errmsg.str()};
  }

  metadata_.relative_file_paths.push_back(storage_->get_relative_file_path());

  // Re-register all topics since we rolled-over to a new bagfile.
  for (const auto & topic : topics_names_to_info_) {
    storage_->create_topic(topic.second.topic_metadata);
  }
}

void SequentialCompressionWriter::compress_message(
  BaseCompressorInterface & compressor,
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  compressor.compress_serialized_bag_message(message.get());
}

void SequentialCompressionWriter::write(
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message)
{
  // If the compression mode is FILE, write as normal here.  Compressing files doesn't
  // occur until after the bag file is split.
  // If the compression mode is MESSAGE, push the message into a queue that will be handled
  // by the compression threads.
  if (compression_options_.compression_mode == CompressionMode::FILE) {
    SequentialWriter::write(message);
  } else {
    std::lock_guard<std::mutex> lock(compressor_mutex_);
    while (compressor_message_queue_.size() > compression_options_.compression_queue_size) {
      compressor_message_queue_.pop();
    }
    compressor_message_queue_.push(message);
    compressor_condition_.notify_one();
  }
}

bool SequentialCompressionWriter::should_split_bagfile()
{
  if (max_bagfile_size_ == rosbag2_storage::storage_interfaces::MAX_BAGFILE_SIZE_NO_SPLIT) {
    return false;
  } else {
    std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
    return storage_->get_bagfile_size() > max_bagfile_size_;
  }
}

}  // namespace rosbag2_compression

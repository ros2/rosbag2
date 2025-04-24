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
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#include "rcpputils/asserts.hpp"

#include "rosbag2_cpp/info.hpp"

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "logging.hpp"
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/resource.h>
#endif

namespace fs = std::filesystem;

namespace rosbag2_compression
{

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
  SequentialCompressionWriter::close();
}

void SequentialCompressionWriter::compression_thread_fn()
{
  if (compression_options_.thread_priority) {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
      "Setting compression thread priority to "
        << *compression_options_.thread_priority);
#ifdef _WIN32
    // This must match THREAD_PRIORITY_IDLE, THREAD_PRIORITY_LOWEST...
    int wanted_thread_priority = *compression_options_.thread_priority;
    if (!SetThreadPriority(GetCurrentThread(), wanted_thread_priority)) {
      ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
        "Could not set thread priority of compression thread to: " << wanted_thread_priority <<
          ". Error code: " << GetLastError());
    } else {
      auto detected_thread_priority = GetThreadPriority(GetCurrentThread());
      if (detected_thread_priority == THREAD_PRIORITY_ERROR_RETURN) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
          "Failed to get current thread priority. Error code: " << GetLastError());
      } else if (wanted_thread_priority != detected_thread_priority) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
          "Could not set thread priority of compression thread to: " <<
            wanted_thread_priority << ". Detected thread priority: " << detected_thread_priority);
      }
    }
#else
    int wanted_nice_value = *compression_options_.thread_priority;

    errno = 0;
    int cur_nice_value = getpriority(PRIO_PROCESS, 0);
    if (cur_nice_value == -1 && errno != 0) {
      ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
        "Could not set nice value of compression thread to: " << wanted_nice_value <<
          " : Could not determine cur nice value");
    } else {
      int new_nice_value = nice(wanted_nice_value - cur_nice_value);
      if ((new_nice_value == -1 && errno != 0)) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
          "Could not set nice value of compression thread to: " << wanted_nice_value <<
            ". Error : " << std::strerror(errno));
      }
    }
#endif
  }

  // Every thread needs to have its own compression context for thread safety.
  auto compressor = compression_factory_->create_compressor(
    compression_options_.compression_format);
  rcpputils::check_true(compressor != nullptr, "Could not create compressor.");

  while (true) {
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message;
    std::string closed_file_relative_to_bag;
    {
      std::unique_lock<std::mutex> lock(compressor_queue_mutex_);
      // *INDENT-OFF*
      compressor_condition_.wait(
        lock,
        [&] {
          return !compression_is_running_ ||
                 !compressor_message_queue_.empty() ||
                 !compressor_file_queue_.empty();
        });
      // *INDENT-ON*

      if (!compressor_message_queue_.empty()) {
        message = compressor_message_queue_.front();
        compressor_message_queue_.pop();
        compressor_condition_.notify_all();
      } else if (!compressor_file_queue_.empty()) {
        closed_file_relative_to_bag = compressor_file_queue_.front();
        compressor_file_queue_.pop();
      } else if (!compression_is_running_) {
        // I woke up, all work queues are empty, and the main thread has stopped execution. Exit.
        break;
      }
    }

    if (message) {
      auto compressed_message = compress_message(*compressor, message);

      {
        // Now that the message is compressed, it can be written to file using the
        // normal method.
        std::lock_guard<std::recursive_mutex> storage_lock(storage_mutex_);
        SequentialWriter::write(compressed_message);
      }
    } else if (!closed_file_relative_to_bag.empty()) {
      compress_file(*compressor, closed_file_relative_to_bag);

      // Execute callbacks from the base class
      static const std::string compressor_ext = "." + compressor->get_compression_identifier();
      auto closed_file =
        (fs::path(base_folder_) / (closed_file_relative_to_bag + compressor_ext)).generic_string();
      std::string new_file;
      // To determine, a new_file we can't rely on the metadata_.relative_file_paths.back(),
      // because other compressor threads may have already pushed a new item above.
      {
        std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
        auto iter = std::find(
          metadata_.relative_file_paths.begin(), metadata_.relative_file_paths.end(),
          closed_file_relative_to_bag + compressor_ext);
        if (iter != metadata_.relative_file_paths.end()) {
          ++iter;
          if (iter != metadata_.relative_file_paths.end()) {
            new_file = (fs::path(base_folder_) / *iter).generic_string();
          }
        }
      }
      if (!new_file.empty()) {
        // The new_file is empty when we compressed the last file after calling close().
        // Note: We shall not call 'execute_bag_split_callbacks(closed_file, new_file)' for the
        // last compressed file because it will be called inside base class
        // SequentialWriter::close().
        SequentialWriter::execute_bag_split_callbacks(closed_file, new_file);
      }
    }
  }
}

void SequentialCompressionWriter::init_metadata()
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::init_metadata();
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
  if (compression_options_.compression_threads < 1) {
    // This should have already been set to something reasonable prior to this
    // point, but we'll double-check just to make sure.
    auto hardware_threads = std::thread::hardware_concurrency();
    compression_options_.compression_threads = hardware_threads > 0 ? hardware_threads : 1;
  }
  ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
    "setup_compressor_threads: Starting " <<
      compression_options_.compression_threads << " threads");
  {
    std::unique_lock<std::mutex> lock(compressor_queue_mutex_);
    compression_is_running_ = true;
  }

  // This function needs to throw an exception if the compression format is invalid, but because
  // each thread creates its own compressor, we can't actually catch it here if one of the threads
  // fails.  Instead, we'll create a compressor that we don't actually use just so that it will
  // throw an exception if the format is invalid.
  auto compressor = compression_factory_->create_compressor(
    compression_options_.compression_format);
  rcpputils::check_true(compressor != nullptr, "Could not create compressor.");

  for (uint64_t i = 0; i < compression_options_.compression_threads; i++) {
    compression_threads_.emplace_back([&] {compression_thread_fn();});
  }
}

void SequentialCompressionWriter::stop_compressor_threads()
{
  if (!compression_threads_.empty()) {
    ROSBAG2_COMPRESSION_LOG_DEBUG("Waiting for compressor threads to finish.");
    {
      std::unique_lock<std::mutex> lock(compressor_queue_mutex_);
      compression_is_running_ = false;
    }
    compressor_condition_.notify_all();
    for (auto & thread : compression_threads_) {
      thread.join();
    }
    compression_threads_.clear();
  }
}

void SequentialCompressionWriter::open(
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  // Note. close and open methods protected with mutex on upper rosbag2_cpp::writer level.
  if (this->is_open_) {
    return;  // The writer already opened.
  }
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::open(storage_options, converter_options);
  setup_compression();
  this->is_open_ = true;
}

void SequentialCompressionWriter::close()
{
  // Note. close and open methods protected with mutex on upper rosbag2_cpp::writer level.
  if (!this->is_open_.exchange(false)) {
    return;  // The writer is not open
  }
  if (!base_folder_.empty()) {
    // Reset may be called before initializing the compressor (ex. bad options).
    // We compress the last file only if it hasn't been compressed earlier (ex. in split_bagfile()).
    if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE &&
      should_compress_last_file_)
    {
      std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
      std::lock_guard<std::mutex> compressor_lock(compressor_queue_mutex_);
      try {
        // Storage must be closed before file can be compressed.
        if (use_cache_) {
          // destructor will flush message cache
          cache_consumer_.reset();
          message_cache_.reset();
        }
        finalize_metadata();
        if (storage_) {
          storage_->update_metadata(metadata_);
          storage_.reset();  // Storage will be closed in storage_ destructor
        }

        if (!metadata_.relative_file_paths.empty()) {
          std::string file = metadata_.relative_file_paths.back();
          compressor_file_queue_.push(file);
          compressor_condition_.notify_one();
        }
      } catch (const std::runtime_error & e) {
        ROSBAG2_COMPRESSION_LOG_WARN_STREAM("Could not compress the last bag file.\n" << e.what());
      }
    }
  }
  stop_compressor_threads();  // Note: The metadata_.relative_file_paths will be updated with
  // compressed filename when compressor threads will finish.
  SequentialWriter::close();
}

void SequentialCompressionWriter::create_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::create_topic(topic_with_type);
}

void SequentialCompressionWriter::create_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type,
  const rosbag2_storage::MessageDefinition & message_definition)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::create_topic(topic_with_type, message_definition);
}

void SequentialCompressionWriter::remove_topic(
  const rosbag2_storage::TopicMetadata & topic_with_type)
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  SequentialWriter::remove_topic(topic_with_type);
}

void SequentialCompressionWriter::compress_file(
  BaseCompressorInterface & compressor,
  const std::string & file_relative_to_bag)
{
  const auto file_relative_to_pwd = fs::path(base_folder_) / file_relative_to_bag;
  ROSBAG2_COMPRESSION_LOG_INFO_STREAM("Compressing file: " << file_relative_to_pwd.string());

  if (fs::exists(file_relative_to_pwd) &&
    fs::file_size(file_relative_to_pwd) > 0u)
  {
    const auto compressed_uri = compressor.compress_uri(file_relative_to_pwd.string());
    const auto relative_compressed_uri = fs::path(compressed_uri).filename();
    {
      // After we've compressed the file, replace the name in the file list with the new name.
      // Must search for the entry because other threads may have changed the order of the vector
      // and invalidated any index or iterator we held to it.
      std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
      const auto iter = std::find(
        metadata_.relative_file_paths.begin(),
        metadata_.relative_file_paths.end(),
        file_relative_to_bag);
      if (iter != metadata_.relative_file_paths.end()) {
        *iter = relative_compressed_uri.string();
      } else {
        ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
          "Failed to find path to uncompressed bag: \"" << file_relative_to_pwd.string() <<
            "\"; this shouldn't happen.");
      }
    }

    if (std::error_code ec;!fs::remove(file_relative_to_pwd, ec)) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
        "Failed to remove original pre-compressed bag file: \"" <<
          file_relative_to_pwd.string() << "\"." << ec.message() <<
          "This should never happen - but execution " <<
          "will not be halted because the compressed output was successfully created.");
    }
  } else {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
      "Removing last file: \"" << file_relative_to_pwd.string() <<
        "\" because it either is empty or does not exist.");
  }
}

void SequentialCompressionWriter::split_bagfile()
{
  std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
  std::lock_guard<std::mutex> compressor_lock(compressor_queue_mutex_);

  // Grab last file before calling common splitting logic, which pushes the new filename
  const auto last_file_relative_to_bag = metadata_.relative_file_paths.back();
  const auto new_file = SequentialWriter::split_bagfile_local(false);

  // If we're in FILE compression mode, push this file's name on to the queue so another
  // thread will handle compressing it.  If not, we can just carry on.
  if (compression_options_.compression_mode == rosbag2_compression::CompressionMode::FILE) {
    compressor_file_queue_.push(last_file_relative_to_bag);
    compressor_condition_.notify_one();
  } else {
    auto last_file = (fs::path(base_folder_) / last_file_relative_to_bag).generic_string();
    SequentialWriter::execute_bag_split_callbacks(last_file, new_file);
  }

  if (!storage_) {
    // Add a check to make sure reset() does not compress the file again if we couldn't load the
    // storage plugin.
    should_compress_last_file_ = false;
  }
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage>
SequentialCompressionWriter::compress_message(
  BaseCompressorInterface & compressor,
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  auto compressed_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  compressed_message->recv_timestamp = message->recv_timestamp;
  compressed_message->send_timestamp = message->send_timestamp;
  compressed_message->topic_name = message->topic_name;
  compressor.compress_serialized_bag_message(message.get(), compressed_message.get());
  return compressed_message;
}

void SequentialCompressionWriter::write(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> message)
{
  // If the compression mode is FILE, write as normal here.  Compressing files doesn't
  // occur until after the bag file is split.
  // If the compression mode is MESSAGE, push the message into a queue that will be handled
  // by the compression threads.
  if (compression_options_.compression_mode == CompressionMode::FILE) {
    std::lock_guard<std::recursive_mutex> lock(storage_mutex_);
    SequentialWriter::write(message);
  } else {
    std::unique_lock<std::mutex> lock(compressor_queue_mutex_);
    while (compressor_message_queue_.size() > compression_options_.compression_queue_size &&
      compression_options_.compression_queue_size > 0u)
    {
      // Drop the message if the queue is full and the compression queue size is set.
      message = compressor_message_queue_.front();
      ROSBAG2_COMPRESSION_LOG_WARN_STREAM(
        "Dropping message on topic:" << message->topic_name <<
          "from compression queue because it is full. Queue size: " <<
          compressor_message_queue_.size());
      compressor_message_queue_.pop();
    }

    // If no message should be dropped and the queue has still messages,
    // compress and write immediately
    if (compression_options_.compression_queue_size == 0u &&
      compressor_message_queue_.size() > compression_options_.compression_threads)
    {
      // *INDENT-OFF*
      compressor_condition_.wait(
        lock,
        [&] {
          return !compression_is_running_ ||
                 compressor_message_queue_.size() <= compression_options_.compression_threads;
        });
      // *INDENT-ON*
    }

    compressor_message_queue_.push(message);
    compressor_condition_.notify_one();
  }
}

}  // namespace rosbag2_compression

// Copyright 2020, Robotec.ai sp. z o.o.
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

#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "rmw/rmw.h"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "rosbag2_performance_writer_benchmarking/writer_benchmark.hpp"

#ifdef _WIN32
// This is necessary because of a bug in yaml-cpp's cmake
#define YAML_CPP_DLL
// This is necessary because yaml-cpp does not always use dllimport/dllexport consistently
# pragma warning(push)
# pragma warning(disable:4251)
# pragma warning(disable:4275)
#endif
#include "yaml-cpp/yaml.h"
#ifdef _WIN32
# pragma warning(pop)
#endif

using namespace std::chrono_literals;

static rcutils_allocator_t allocator = rcutils_get_default_allocator();

WriterBenchmark::WriterBenchmark(const std::string & name)
: rclcpp::Node(name)
{
  const std::string default_bag_folder("/tmp/rosbag2_test");
  this->declare_parameter("frequency", 100);
  this->declare_parameter("max_count", 1000);
  this->declare_parameter("size", 1000000);
  this->declare_parameter("instances", 1);
  this->declare_parameter("max_cache_size", 10000000);
  this->declare_parameter("max_bag_size", 0);
  this->declare_parameter("db_folder", default_bag_folder);
  this->declare_parameter("results_file", default_bag_folder + "/results.csv");
  this->declare_parameter("storage_config_file", "");
  this->declare_parameter("compression_format", "");
  this->declare_parameter("compression_queue_size", 1);
  this->declare_parameter("compression_threads", 0);

  this->get_parameter("frequency", config_.frequency);
  if (config_.frequency == 0) {
    RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
    rclcpp::shutdown(nullptr, "frequency error");
    return;
  }

  storage_options_.storage_id = "sqlite3";
  this->get_parameter("max_cache_size", storage_options_.max_cache_size);
  this->get_parameter("db_folder", storage_options_.uri);
  this->get_parameter("storage_config_file", storage_options_.storage_config_uri);
  this->get_parameter("max_bag_size", storage_options_.max_bagfile_size);

  this->get_parameter("results_file", results_file_);
  this->get_parameter("max_count", config_.max_count);
  this->get_parameter("size", config_.message_size);
  this->get_parameter("instances", instances_);
  this->get_parameter("compression_format", compression_format_);
  this->get_parameter("compression_queue_size", compression_queue_size_);
  this->get_parameter("compression_threads", compression_threads_);

  create_producers(config_);
  create_writer();
}

void WriterBenchmark::start_benchmark()
{
  RCLCPP_INFO(get_logger(), "Starting");
  start_producers();
  while (rclcpp::ok()) {
    int count = 0;
    unsigned int complete_count = 0;

    // TODO(adamdbrw) Performance can be improved. Use conditional variables
    for (auto & queue : queues_) {
      if (queue->is_complete() && queue->is_empty()) {
        ++complete_count;
      }

      if (!queue->is_empty()) {  // behave as if we received the message.
        auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

        // The pointer memory is owned by the producer until past the termination of the while loop.
        // Note this ownership model should be changed if we want to generate messages on the fly
        auto byte_ma_message = queue->pop_and_return();

        // The compressor may resize this array, so it needs to be initialized with
        // rcutils_uint8_array_init to ensure the allocator is set properly.
        auto msg_array = new rcutils_uint8_array_t;
        *msg_array = rcutils_get_zero_initialized_uint8_array();
        int error = rcutils_uint8_array_init(msg_array, byte_ma_message->data.size(), &allocator);
        if (error != RCUTILS_RET_OK) {
          throw std::runtime_error(
                  "Error allocating resources for serialized message: " +
                  std::string(rcutils_get_error_string().str));
        }
        // The compressor may modify this buffer in-place, so it should take ownership of it.
        std::move(
          byte_ma_message->data.data(),
          byte_ma_message->data.data() + byte_ma_message->data.size(),
          msg_array->buffer);
        auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(msg_array);
        serialized_data->buffer_length = byte_ma_message->data.size();

        message->serialized_data = serialized_data;

        rcutils_time_point_value_t time_stamp;
        error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK) {
          RCLCPP_ERROR_STREAM(
            get_logger(), "Error getting current time. Error:" <<
              rcutils_get_error_string().str);
        }
        message->time_stamp = time_stamp;
        message->topic_name = queue->topic_name();

        try {
          writer_->write(message);
        } catch (const std::runtime_error & e) {
          RCLCPP_ERROR_STREAM(get_logger(), "Failed to record: " << e.what());
        }
        ++count;
      }
    }
    if (complete_count == queues_.size()) {
      break;
    }

    std::this_thread::sleep_for(1ms);
  }

  for (auto & prod_thread : producer_threads_) {
    prod_thread.join();
  }

  writer_->reset();
  write_results();
}

int WriterBenchmark::get_message_count_from_metadata() const
{
  int total_recorded_count = 0;
  std::string metadata_filename(rosbag2_storage::MetadataIo::metadata_filename);
  std::string metadata_path = storage_options_.uri + "/" + metadata_filename;
  try {
    YAML::Node yaml_file = YAML::LoadFile(metadata_path);
    total_recorded_count = yaml_file["rosbag2_bagfile_information"]["message_count"].as<int>();
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error(
            std::string("Exception on parsing metadata file to get total message count: ") +
            metadata_path + " " +
            ex.what());
  }
  return total_recorded_count;
}

void WriterBenchmark::write_results() const
{
  int total_recorded_count = get_message_count_from_metadata();
  auto total_messages_sent = config_.max_count * producers_.size();
  float percentage_recorded = static_cast<float>(total_recorded_count * 100.0f) /
    total_messages_sent;

  RCLCPP_INFO_STREAM(
    get_logger(), "Percentage of all messages that were successfully recorded: " <<
      percentage_recorded);

  bool new_file = false;
  { // test if file exists - we want to write a csv header after creation if not
    // use std::filesystem when switching to C++17
    std::ifstream test_existence(results_file_);
    if (!test_existence) {
      new_file = true;
    }
  }

  // append, we want to accumulate results from multiple runs
  std::ofstream output_file(results_file_, std::ios_base::app);
  if (!output_file.is_open()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not open file " << results_file_);
    return;
  }

  if (new_file) {
    output_file << "instances frequency message_size cache_size total_messages_sent ";
    output_file << "percentage_recorded\n";
  }

  // configuration of the test. TODO(adamdbrw) wrap into a dict and define << operator.
  output_file << instances_ << " ";
  output_file << config_.frequency << " ";
  output_file << config_.message_size << " ";
  output_file << storage_options_.max_cache_size << " ";
  output_file << total_messages_sent << " ";

  // results of the test. Use std::setprecision if preferred
  output_file << percentage_recorded << std::endl;
}

void WriterBenchmark::create_producers(const ProducerConfig & config)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "\nWriterBenchmark: creating " << instances_ <<
      " message producers with frequency " << config.frequency <<
      " and message size in bytes " << config.message_size <<
      ". Cache is " << storage_options_.max_cache_size <<
      ". Each will send " << config.max_count <<
      " messages before terminating");
  const unsigned int queue_max_size = 10;
  for (unsigned int i = 0; i < instances_; ++i) {
    std::string topic = "/writer_benchmark/producer " + std::to_string(i);
    auto queue = std::make_shared<ByteMessageQueue>(queue_max_size, topic);
    queues_.push_back(queue);
    producers_.push_back(std::make_unique<ByteProducer>(config, queue));
  }
}

// TODO(adamdbrw) extend to other writers - based on parametrization
// Also, add an option to configure compression
void WriterBenchmark::create_writer()
{
  if (!compression_format_.empty()) {
    rosbag2_compression::CompressionOptions compression_options{
      compression_format_, rosbag2_compression::CompressionMode::MESSAGE,
      compression_queue_size_, compression_threads_};

    writer_ = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);
  } else {
    writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  }

  // TODO(adamdbrw) generalize if converters are to be included in benchmarks
  std::string serialization_format = rmw_get_serialization_format();
  writer_->open(storage_options_, {serialization_format, serialization_format});

  for (const auto & queue : queues_) {
    rosbag2_storage::TopicMetadata topic;
    topic.name = queue->topic_name();
    // TODO(adamdbrw) - replace with something more general if needed
    topic.type = "std_msgs::msgs::ByteMultiArray";
    topic.serialization_format = serialization_format;
    writer_->create_topic(topic);
  }
}

void WriterBenchmark::start_producers()
{
  for (auto & producer : producers_) {
    producer_threads_.push_back(std::thread(&ByteProducer::run, producer.get()));
  }
}

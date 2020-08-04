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
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "rosbag2_performance_writer_benchmarking/writer_benchmark.hpp"

using namespace std::chrono_literals;

WriterBenchmark::WriterBenchmark()
: rclcpp::Node("writer_benchmark")
{
  const std::string default_bag_folder("/tmp/rosbag2_test");
  this->declare_parameter("frequency", 100);
  this->declare_parameter("max_count", 1000);
  this->declare_parameter("size", 1000000);
  this->declare_parameter("instances", 1);
  this->declare_parameter("max_cache_size", 1);
  this->declare_parameter("db_folder", default_bag_folder);
  this->declare_parameter("results_file", default_bag_folder + "/results.csv");

  this->get_parameter("frequency", config_.frequency);
  if (config_.frequency == 0) {
    RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
    rclcpp::shutdown(nullptr, "frequency error");
    return;
  }

  this->get_parameter("max_cache_size", max_cache_size_);
  this->get_parameter("db_folder", db_folder_);
  this->get_parameter("results_file", results_file_);
  this->get_parameter("max_count", config_.max_count);
  this->get_parameter("size", config_.message_size);
  this->get_parameter("instances", instances_);

  create_producers(config_);
  create_writer();
}

void WriterBenchmark::start_benchmark()
{
  RCLCPP_INFO(get_logger(), "Starting. A dot is a write, an X is a miss");
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
        auto serialized_data = std::make_shared<rcutils_uint8_array_t>();

        // The pointer memory is owned by the producer until past the termination of the while loop.
        // Note this ownership model should be changed if we want to generate messages on the fly
        auto byte_ma_message = queue->pop_and_return();

        serialized_data->buffer = reinterpret_cast<uint8_t *>(byte_ma_message->data.data());
        serialized_data->buffer_length = byte_ma_message->data.size();
        serialized_data->buffer_capacity = byte_ma_message->data.size();

        message->serialized_data = serialized_data;

        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
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
        std::cerr << "." << std::flush;
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

  unsigned int total_missed_messages = 0;
  for (const auto & queue : queues_) {
    total_missed_messages += queue->get_missed_elements_count();
  }
  write_results(total_missed_messages);
}

void WriterBenchmark::write_results(const unsigned int & total_missed) const
{
  unsigned int total_messages_sent = config_.max_count * producers_.size();
  float percentage_recorded = 100.0f - static_cast<float>(total_missed * 100.0f) /
    total_messages_sent;

  RCLCPP_INFO(get_logger(), "\nWriterBenchmark terminating");
  RCLCPP_INFO_STREAM(get_logger(), "Total missed messages: " << total_missed);
  RCLCPP_INFO_STREAM(
    get_logger(), "Percentage of all message that was successfully recorded: " <<
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
    output_file << "total_messages_missed percentage_recorded\n";
  }

  // configuration of the test. TODO(adamdbrw) wrap into a dict and define << operator.
  output_file << instances_ << " ";
  output_file << config_.frequency << " ";
  output_file << config_.message_size << " ";
  output_file << max_cache_size_ << " ";
  output_file << total_messages_sent << " ";

  // results of the test. Use std::setprecision if preferred
  output_file << total_missed << " ";
  output_file << percentage_recorded << std::endl;
}

void WriterBenchmark::create_producers(const ProducerConfig & config)
{
  RCLCPP_INFO_STREAM(
    get_logger(), "\nWriterBenchmark: creating " << instances_ <<
      " message producers with frequency " << config.frequency <<
      " and message size in bytes " << config.message_size <<
      ". Cache is " << max_cache_size_ << ". Each will send " << config.max_count <<
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
  writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  rosbag2_cpp::StorageOptions storage_options{};
  storage_options.uri = db_folder_;
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;
  storage_options.max_cache_size = max_cache_size_;

  // TODO(adamdbrw) generalize if converters are to be included in benchmarks
  std::string serialization_format = rmw_get_serialization_format();
  writer_->open(storage_options, {serialization_format, serialization_format});

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

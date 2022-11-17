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
#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rmw/rmw.h"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "rosbag2_performance_benchmarking/config_utils.hpp"
#include "rosbag2_performance_benchmarking/result_utils.hpp"
#include "rosbag2_performance_benchmarking/writer_benchmark.hpp"

using namespace std::chrono_literals;

static rcutils_allocator_t allocator = rcutils_get_default_allocator();

WriterBenchmark::WriterBenchmark(const std::string & name)
: rclcpp::Node(name)
{
  RCLCPP_INFO(get_logger(), "WriterBenchmark parsing configurations");
  configurations_ = config_utils::publisher_groups_from_node_parameters(*this);
  if (configurations_.empty()) {
    RCLCPP_ERROR(get_logger(), "No publishers/producers found in node parameters");
    return;
  }

  bag_config_ = config_utils::bag_config_from_node_parameters(*this);

  this->declare_parameter("results_file", bag_config_.storage_options.uri + "/results.csv");
  this->get_parameter("results_file", results_file_);

  RCLCPP_INFO(get_logger(), "configuration parameters processed");

  create_producers();
  create_writer();
}

void WriterBenchmark::start_benchmark()
{
  RCLCPP_INFO(get_logger(), "Starting the WriterBenchmark");
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

        auto serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
          msg_array,
          [this](rcutils_uint8_array_t * msg) {
            int error = rcutils_uint8_array_fini(msg);
            delete msg;
            if (error != RCUTILS_RET_OK) {
              RCLCPP_ERROR_STREAM(
                get_logger(),
                "Leaking memory. Error: " << rcutils_get_error_string().str);
            }
          });

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
  writer_->close();

  result_utils::write_benchmark_results(configurations_, bag_config_, results_file_);
}

void WriterBenchmark::create_producers()
{
  RCLCPP_INFO_STREAM(get_logger(), "creating producers");
  for (const auto & c : configurations_) {
    RCLCPP_INFO_STREAM(
      get_logger(), "\nWriterBenchmark: creating " << c.count <<
        " message producers with frequency " << c.producer_config.frequency <<
        " and message size in bytes " << c.producer_config.message_size <<
        " for topic root of " << c.topic_root <<
        ". Each will send " << c.producer_config.max_count <<
        " messages before terminating");
    const unsigned int queue_max_size = 10;
    for (unsigned int i = 0; i < c.count; ++i) {
      std::string topic = c.topic_root + std::to_string(i);
      auto queue = std::make_shared<ByteMessageQueue>(queue_max_size, topic);
      queues_.push_back(queue);
      producers_.push_back(
        std::make_unique<ByteProducer>(
          c.producer_config,
          [] { /* empty lambda */},
          [queue](std::shared_ptr<std_msgs::msg::ByteMultiArray> msg) {
            queue->push(msg);
          },
          [queue] {
            queue->set_complete();
          }));
    }
  }
}

void WriterBenchmark::create_writer()
{
  if (!bag_config_.compression_format.empty()) {
    rosbag2_compression::CompressionOptions compression_options{
      bag_config_.compression_format, rosbag2_compression::CompressionMode::MESSAGE,
      bag_config_.compression_queue_size, bag_config_.compression_threads};

    writer_ = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(
      compression_options);
  } else {
    writer_ = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  }

  // TODO(adamdbrw) generalize if converters are to be included in benchmarks
  std::string serialization_format = rmw_get_serialization_format();
  writer_->open(bag_config_.storage_options, {serialization_format, serialization_format});

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto bench = std::make_shared<WriterBenchmark>("rosbag2_performance_benchmarking_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bench);

  // The benchmark has its own control loop but uses spinning for parameters
  std::thread spin_thread([&executor]() {executor.spin();});
  bench->start_benchmark();
  RCLCPP_INFO(bench->get_logger(), "Benchmark terminated");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

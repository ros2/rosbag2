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

#include "rmw/rmw.h"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "rosbag2_transport/storage_options.hpp"

#include "writer_benchmark.hpp"

using namespace std::chrono_literals;

WriterBenchmark::WriterBenchmark()
  : rclcpp::Node("writer_benchmark")
{
  this->declare_parameter("frequency", 100);
  this->declare_parameter("max_count", 1000);
  this->declare_parameter("size", 1000000);
  this->declare_parameter("instances", 1);
  this->declare_parameter("max_cache_size", 1);
  this->declare_parameter("db_folder", std::string("/tmp/rosbag2_test"));

  this->get_parameter("frequency", mConfig.frequency);
  if (mConfig.frequency == 0)
  {
      RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "frequency error");
      return;
  }

  this->get_parameter("max_cache_size", mMaxCacheSize);
  this->get_parameter("db_folder", mDbFolder);
  this->get_parameter("max_count", mConfig.max_count);
  this->get_parameter("size", mConfig.message_size);

  unsigned int instances = 1;
  this->get_parameter("instances", instances);

  createProducers(mConfig, instances);
  createWriter();
}

void WriterBenchmark::startBenchmark()
{
  RCLCPP_INFO(get_logger(), "Starting. A dot is a write, an X is a miss");
  startProducers();
  while (rclcpp::ok())
  {
    int count = 0;
    unsigned int completeCount = 0;

    for (auto &queue : mQueues)
    {   // TODO(adamdbrw) Performance can be improved. Use conditional variables
      if (queue->isComplete() && queue->isEmpty()) {
        ++completeCount;
      }

      if (!queue->isEmpty())
      {   // behave as if we received the message.
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
        if (error != RCUTILS_RET_OK)
        {
          RCLCPP_ERROR_STREAM(get_logger(), "Error getting current time. Error:"
            << rcutils_get_error_string().str);
        }
        message->time_stamp = time_stamp;
        message->topic_name = queue->topicName();

        try
        {
          mWriter->write(message);
        } catch (std::runtime_error & e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Failed to record: " << e.what());
        }
        std::cerr << ".";
        ++count;
      }
    }
    if (completeCount == mQueues.size())
      break;

    std::this_thread::sleep_for(1ms);
  }

  for (auto &prodThread : mProducerThreads)
  {
    prodThread.join();
  }

  unsigned int totalMissedMessages = 0;
  for (const auto &queue : mQueues)
  {
    totalMissedMessages += queue->getMissedElementsCount();
  }

  RCLCPP_INFO(get_logger(), "\nWriterBenchmark terminating");
  RCLCPP_INFO_STREAM(get_logger(), "Total missed messages: " << totalMissedMessages);
  RCLCPP_INFO_STREAM(get_logger(), "Percentage of all message that was successfully recorded: "
    << 100.0 - (float)totalMissedMessages * 100.0 / (mConfig.max_count * mProducers.size()));
}

void WriterBenchmark::createProducers(const ProducerConfig &config, unsigned int instances)
{
  RCLCPP_INFO_STREAM(get_logger(), "\nWriterBenchmark: creating " << instances
    << " message producers with frequency " << config.frequency
    << " and message size in bytes " << config.message_size
    << ". Cache is " << mMaxCacheSize << ". Each will send " << config.max_count
    << " messages before terminating");
  const unsigned int queueMaxSize = 10;
  for (unsigned int i = 0; i < instances; ++i)
  {
    std::string topic = "/writer_benchmark/producer " + std::to_string(i);
    auto queue = std::make_shared<ByteMessageQueue>(queueMaxSize, topic);
    mQueues.push_back(queue);
    mProducers.push_back(std::make_unique<ByteProducer>(config, queue));
  }
}

// TODO(adamdbrw) extend to other writers - based on parametrization
// Also, add an option to configure compression
void WriterBenchmark::createWriter()
{
  mWriter = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  rosbag2_transport::StorageOptions storage_options{};
  storage_options.uri = mDbFolder;
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;
  storage_options.max_cache_size = mMaxCacheSize;

  // TODO(adamdbrw) generalize if converters are to be included in benchmarks
  std::string serialization_format = rmw_get_serialization_format();
  mWriter->open(storage_options, {serialization_format, serialization_format});

  for (size_t i = 0; i < mQueues.size(); ++i)
  {
    rosbag2_storage::TopicMetadata topic;
    topic.name = mQueues[i]->topicName();
    // TODO(adamdbrw) - replace with something more general if needed
    topic.type = "std_msgs::msgs::ByteMultiArray";
    topic.serialization_format = serialization_format;
    mWriter->create_topic(topic);
  }
}

void WriterBenchmark::startProducers()
{
  for (auto &producer : mProducers)
  {
    mProducerThreads.push_back(std::thread(&ByteProducer::run, producer.get()));
  }
}

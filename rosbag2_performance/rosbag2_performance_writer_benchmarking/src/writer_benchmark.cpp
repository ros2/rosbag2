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
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "rosbag2_transport/storage_options.hpp"

#include "writer_benchmark.hpp"

using namespace std::chrono_literals;

WriterBenchmark::WriterBenchmark()
  : rclcpp::Node("writer_benchmark")
{
  const std::string defaultBagFolder("/tmp/rosbag2_test");
  this->declare_parameter("frequency", 100);
  this->declare_parameter("max_count", 1000);
  this->declare_parameter("size", 1000000);
  this->declare_parameter("instances", 1);
  this->declare_parameter("max_cache_size", 1);
  this->declare_parameter("db_folder", defaultBagFolder);
  this->declare_parameter("results_file", defaultBagFolder + "/results.csv");

  this->get_parameter("frequency", _config.frequency);
  if (_config.frequency == 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
    rclcpp::shutdown(nullptr, "frequency error");
    return;
  }

  this->get_parameter("max_cache_size", _maxCacheSize);
  this->get_parameter("db_folder", _dbFolder);
  this->get_parameter("results_file", _resultsFile);
  this->get_parameter("max_count", _config.max_count);
  this->get_parameter("size", _config.message_size);
  this->get_parameter("instances", _instances);

  create_producers(_config);
  create_writer();
}

void WriterBenchmark::start_benchmark()
{
  RCLCPP_INFO(get_logger(), "Starting. A dot is a write, an X is a miss");
  start_producers();
  while (rclcpp::ok())
  {
    int count = 0;
    unsigned int completeCount = 0;

    for (auto &queue : _queues)
    {   // TODO(adamdbrw) Performance can be improved. Use conditional variables
      if (queue->is_complete() && queue->is_empty()) {
        ++completeCount;
      }

      if (!queue->is_empty())
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
        message->topic_name = queue->topic_name();

        try
        {
          _writer->write(message);
        } catch (std::runtime_error & e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Failed to record: " << e.what());
        }
        std::cerr << ".";
        ++count;
      }
    }
    if (completeCount == _queues.size())
      break;

    std::this_thread::sleep_for(1ms);
  }

  for (auto &prodThread : _producerThreads)
  {
    prodThread.join();
  }

  unsigned int totalMissedMessages = 0;
  for (const auto &queue : _queues)
  {
    totalMissedMessages += queue->get_missed_elements_count();
  }
  write_results(totalMissedMessages);
}

void WriterBenchmark::write_results(const unsigned int &totalMissed) const
{
  unsigned int totalMessagesSent = _config.max_count * _producers.size();
  float percentageRecorded = 100.0 - static_cast<float>(totalMissed * 100.0) / totalMessagesSent;

  RCLCPP_INFO(get_logger(), "\nWriterBenchmark terminating");
  RCLCPP_INFO_STREAM(get_logger(), "Total missed messages: " << totalMissed);
  RCLCPP_INFO_STREAM(get_logger(), "Percentage of all message that was successfully recorded: "
    << percentageRecorded);

  bool newFile = false;
  { // test if file exists - we want to write a csv header after creation if not
    // use std::filesystem when switching to C++17
    std::ifstream testExistence(_resultsFile);
    if (!testExistence)
    {
      newFile = true;
    }
  }

  // append, we want to accumulate results from multiple runs
  std::ofstream outputFile(_resultsFile, std::ios_base::app);
  if (!outputFile.is_open())
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not open file " << _resultsFile);
    return;
  }

  if (newFile)
  {
    outputFile << "instances frequency message_size cache_size total_messages_sent ";
    outputFile << "total_messages_missed percentage_recorded\n";
  }

  // configuration of the test. TODO(adamdbrw) wrap into a dict and define << operator.
  outputFile << _instances << " ";
  outputFile << _config.frequency << " ";
  outputFile << _config.message_size << " ";
  outputFile << _maxCacheSize << " ";
  outputFile << totalMessagesSent << " ";

  // results of the test. Use std::setprecision if preferred
  outputFile << totalMissed << " ";
  outputFile << percentageRecorded << "\n";
}

void WriterBenchmark::create_producers(const ProducerConfig &config)
{
  RCLCPP_INFO_STREAM(get_logger(), "\nWriterBenchmark: creating " << _instances
    << " message producers with frequency " << config.frequency
    << " and message size in bytes " << config.message_size
    << ". Cache is " << _maxCacheSize << ". Each will send " << config.max_count
    << " messages before terminating");
  const unsigned int queueMaxSize = 10;
  for (unsigned int i = 0; i < _instances; ++i)
  {
    std::string topic = "/writer_benchmark/producer " + std::to_string(i);
    auto queue = std::make_shared<ByteMessageQueue>(queueMaxSize, topic);
    _queues.push_back(queue);
    _producers.push_back(std::make_unique<ByteProducer>(config, queue));
  }
}

// TODO(adamdbrw) extend to other writers - based on parametrization
// Also, add an option to configure compression
void WriterBenchmark::create_writer()
{
  _writer = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  rosbag2_transport::StorageOptions storage_options{};
  storage_options.uri = _dbFolder;
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;
  storage_options.max_cache_size = _maxCacheSize;

  // TODO(adamdbrw) generalize if converters are to be included in benchmarks
  std::string serialization_format = rmw_get_serialization_format();
  _writer->open(storage_options, {serialization_format, serialization_format});

  for (size_t i = 0; i < _queues.size(); ++i)
  {
    rosbag2_storage::TopicMetadata topic;
    topic.name = _queues[i]->topic_name();
    // TODO(adamdbrw) - replace with something more general if needed
    topic.type = "std_msgs::msgs::ByteMultiArray";
    topic.serialization_format = serialization_format;
    _writer->create_topic(topic);
  }
}

void WriterBenchmark::start_producers()
{
  for (auto &producer : _producers)
  {
    _producerThreads.push_back(std::thread(&ByteProducer::run, producer.get()));
  }
}

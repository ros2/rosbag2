#include <chrono>

#include "rmw/rmw.h"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "rosbag2_transport/storage_options.hpp"

#include "writer_benchmark.hpp"

using namespace std::chrono_literals;

WriterBenchmark::WriterBenchmark() : rclcpp::Node("writer_benchmark")
{
  this->declare_parameter("frequency");
  this->declare_parameter("max_count");
  this->declare_parameter("size");
  this->declare_parameter("instances");
  this->declare_parameter("max_cache_size");
  this->declare_parameter("db_folder");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  while (!parameters_client->wait_for_service(1s))
  {
      if (!rclcpp::ok())
      {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  mConfig.frequency = parameters_client->get_parameter("frequency", 100);
  if (mConfig.frequency == 0)
  {
      RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "frequency error");
      return;
  }

  mMaxCacheSize = parameters_client->get_parameter("max_cache_size", 1);
  mDbFolder = parameters_client->get_parameter("db_folder", std::string("/tmp/rosbag2_test"));
  mConfig.max_count = parameters_client->get_parameter("max_count", 1000);
  mConfig.message_size = parameters_client->get_parameter("size", 1000000);

  unsigned int instances = parameters_client->get_parameter("instances", 1);
  createProducers(mConfig, instances);
  createWriter();
}

void WriterBenchmark::startBenchmark()
{
  RCLCPP_INFO(get_logger(), "WriterBenchmark starting. A dot is successful write, an X is a missed message");
  startProducers();
  while (rclcpp::ok())
  {
    int count = 0;
    unsigned int completeCount = 0;
    for (size_t i = 0; i < mQueues.size(); ++i)
    { //TODO use conditional variables to avoid locks
      if (mQueues.at(i)->isComplete() && mQueues.at(i)->isEmpty())
        completeCount++;

      if (!mQueues.at(i)->isEmpty())
      { //behave as if we received the message.
        auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        auto serialized_data = std::make_shared<rcutils_uint8_array_t>();

        //TODO: Bad in general case
        //Data only valid for message lifetime!
        auto byte_ma_message = mQueues[i]->pop_and_return();

        serialized_data->buffer = (uint8_t *)byte_ma_message.data.data();
        serialized_data->buffer_length = byte_ma_message.data.size();
        serialized_data->buffer_capacity = byte_ma_message.data.size();

        message->serialized_data = serialized_data;

        rcutils_time_point_value_t time_stamp;
        int error = rcutils_system_time_now(&time_stamp);
        if (error != RCUTILS_RET_OK)
        {
          RCLCPP_ERROR_STREAM(get_logger(), "Error getting current time. Error:" << rcutils_get_error_string().str);
        }
        message->time_stamp = time_stamp;
        message->topic_name = mQueues[i]->topicName();
        mWriter->write(message);
        std::cerr << ".";
        count++;
      }
    }
    //RCLCPP_INFO_STREAM(get_logger(), "Wrote " << count << " messages");
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

  RCLCPP_INFO(get_logger(), "/nWriterBenchmark terminating");
  RCLCPP_INFO_STREAM(get_logger(), "Total missed messages: " << totalMissedMessages);
  RCLCPP_INFO_STREAM(get_logger(), "Percentage of all message that was successfully recorded: " << 100.0 - (float)totalMissedMessages*100.0/(mConfig.max_count*mProducers.size()));
}

void WriterBenchmark::createProducers(const ProducerConfig &config, unsigned int instances)
{
  RCLCPP_INFO_STREAM(get_logger(), "/nWriterBenchmark: creating " << instances << " message producers with frequency "
    << config.frequency << " and message size in bytes" << config.message_size << ". Cache is " << mMaxCacheSize << ". Each will send " << config.max_count << " messages before terminating");
  const unsigned int queueMaxSize = 10;
  for (unsigned int i = 0; i < instances; ++i)
  {
    std::string topic_name = "/writer_benchmark/producer " + std::to_string(i);
    auto queue = std::make_shared<MessageQueue<std_msgs::msg::ByteMultiArray>>(queueMaxSize, topic_name);
    auto producer = std::make_shared<ByteProducer>(config, queue);
    mQueues.push_back(queue);
    mProducers.push_back(producer);
  }
}

void WriterBenchmark::createWriter()
{
  mWriter = std::make_shared<rosbag2_cpp::writers::SequentialWriter>();
  rosbag2_transport::StorageOptions storage_options{};
  storage_options.uri = mDbFolder;
  storage_options.storage_id = "sqlite3";
  storage_options.max_bagfile_size = 0;
  storage_options.max_cache_size = mMaxCacheSize;

  std::string serialization_format = rmw_get_serialization_format();
  mWriter->open(storage_options, {serialization_format, serialization_format});

  for (size_t i = 0; i < mQueues.size(); ++i)
  {
    rosbag2_storage::TopicMetadata topic;
    topic.name = mQueues[i]->topicName();
    //TODO
    topic.type = "std_msgs::msgs::ByteMultiArray";
    topic.serialization_format = serialization_format;
    mWriter->create_topic(topic);
  }
}

void WriterBenchmark::startProducers()
{
  for (auto producer : mProducers)
  {
    //RCLCPP_INFO(get_logger(), "starting a producer ");
    mProducerThreads.push_back(std::thread(&ByteProducer::run, producer.get()));
  }
}

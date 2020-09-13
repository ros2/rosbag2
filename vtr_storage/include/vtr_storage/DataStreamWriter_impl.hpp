// description
#pragma once

#include "vtr_storage/DataStreamWriter.hpp"

namespace vtr {
namespace storage {

template<typename MessageType>
DataStreamWriter<MessageType>::DataStreamWriter(const std::string &data_directory_string,
                                   const std::string &stream_name, bool append)
    : DataStreamWriterBase(data_directory_string, stream_name, append) {
  tm_ = createTopicMetadata();
}

template<typename MessageType>
DataStreamWriter<MessageType>::~DataStreamWriter() { close(); }

template<typename MessageType>
void DataStreamWriter<MessageType>::open() {
  if (!this->opened_) {
    writer_ = std::make_shared<SequentialAppendWriter>();
    writer_->open(this->storage_options_, this->converter_options_);
    if (!this->append_) writer_->create_topic(tm_);
    this->opened_ = true;
  }
}

template<typename MessageType>
void DataStreamWriter<MessageType>::close() {
  writer_.reset();
  this->opened_ = false;
}

template<typename MessageType>
rosbag2_storage::TopicMetadata DataStreamWriter<MessageType>::createTopicMetadata() {
  // ToDo: create topic based on topic name
  rosbag2_storage::TopicMetadata tm;
  tm.name = "/my/test/topic";
  tm.type = "test_msgs/msg/BasicTypes";
  tm.serialization_format = "cdr";
  return tm;
}

template<typename MessageType>
int32_t DataStreamWriter<MessageType>::write(const std::any &anytype_message) {
  if (!this->opened_) open();
  try {
    auto message = std::any_cast<MessageType>(anytype_message);
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto ret = rcutils_system_time_now(&bag_message->time_stamp);
    if (ret != RCL_RET_OK) {
      throw std::runtime_error("couldn't assign time rosbag message");
    }
    rclcpp::SerializedMessage serialized_msg;
    this->serialization_.serialize_message(&message, &serialized_msg);

    bag_message->topic_name = tm_.name;
    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        &serialized_msg.get_rcl_serialized_message(),
        [](rcutils_uint8_array_t * /* data */) {});

    writer_->write(bag_message);
    return writer_->get_last_inserted_id();
  } catch (const std::bad_any_cast& e) {
      std::stringstream ss;
      ss << "Any cast failed in writing data in DataStreamWriter. Error: " << e.what();
      throw std::runtime_error(ss.str());
  }
}

}  // namespace storage
}  // namespace vtr
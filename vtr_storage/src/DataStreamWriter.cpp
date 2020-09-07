// description
#include "vtr_storage/DataStreamWriter.hpp"

namespace vtr {
namespace storage {

DataStreamWriter::DataStreamWriter(const std::string &data_directory_string,
                                   const std::string &stream_name, bool append)
    : DataStreamBase(data_directory_string, stream_name), append_(append) {
  tm_ = createTopicMetadata();
}

DataStreamWriter::~DataStreamWriter() { close(); }

void DataStreamWriter::open() {
  if (!opened_) {
    writer_ = std::make_shared<SequentialAppendWriter>();
    writer_->open(storage_options_, converter_options_);
    if (!append_) writer_->create_topic(tm_);
    opened_ = true;
  }
}

void DataStreamWriter::close() {
  writer_.reset();
  opened_ = false;
}

rosbag2_storage::TopicMetadata DataStreamWriter::createTopicMetadata() {
  // ToDo: create topic based on topic name
  rosbag2_storage::TopicMetadata tm;
  tm.name = "/my/test/topic";
  tm.type = "test_msgs/msg/BasicTypes";
  tm.serialization_format = "cdr";
  return tm;
}

int32_t DataStreamWriter::write(const TestMsgT &message) {
  if (!opened_) open();

  auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  auto ret = rcutils_system_time_now(&bag_message->time_stamp);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error("couldn't assign time rosbag message");
  }
  rclcpp::SerializedMessage serialized_msg;
  serialization_.serialize_message(&message, &serialized_msg);

  bag_message->topic_name = tm_.name;
  bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &serialized_msg.get_rcl_serialized_message(),
      [](rcutils_uint8_array_t * /* data */) {});

  writer_->write(bag_message);
  return writer_->get_last_inserted_id();
}

}  // namespace storage
}  // namespace vtr
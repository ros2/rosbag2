// description
#include "rosbag2_extensions/DataStreamWriter.hpp"

namespace rosbag2_extensions
{

DataStreamWriter::DataStreamWriter(const std::string &data_directory,const std::string &stream_name) :
DataStreamBase(data_directory, stream_name) {
  tm_ = createTopicMetadata();
}

DataStreamWriter::~DataStreamWriter() {
    close();
}

void DataStreamWriter::open() {
    if(!opened_) {
        rcpputils::fs::create_directories(data_directory_);
        writer_.open(storage_options_, converter_options_);
        writer_.create_topic(tm_);
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

void DataStreamWriter::write(const TestMsgT &message) {
    if(!opened_) open();

    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    auto ret = rcutils_system_time_now(&bag_message->time_stamp);
    if (ret != RCL_RET_OK) {
        throw std::runtime_error("couldn't assign time rosbag message");
    }
    rclcpp::SerializedMessage serialized_msg;
    bag_message->topic_name = tm_.name;
    serialization_.serialize_message(&message, &serialized_msg);

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        &serialized_msg.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

    writer_.write(bag_message);
}


}
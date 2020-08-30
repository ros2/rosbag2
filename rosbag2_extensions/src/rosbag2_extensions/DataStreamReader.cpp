// description
#include "rosbag2_extensions/DataStreamReader.hpp"

namespace rosbag2_extensions
{

DataStreamReader::DataStreamReader(const std::string &data_directory,const std::string &stream_name) :
DataStreamBase(data_directory, stream_name) {}

DataStreamReader::~DataStreamReader() {
    close();
}

void DataStreamReader::close() {
    reader_.reset();
    opened_ = false;
}

void DataStreamReader::openAndGetMessageType() {
    if(opened_ == false) {
        reader_.open(storage_options_, converter_options_);
        opened_ = true;
    }
    // ToDo: get message type, specialize serialization_ based on message type?
}

std::shared_ptr<TestMsgT> DataStreamReader::readAtIndex(uint32_t index) {
    openAndGetMessageType();
    auto bag_message = reader_.read_at_index(index);
    auto extracted_test_msg = std::make_shared<TestMsgT>();
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    serialization_.deserialize_message(&extracted_serialized_msg, extracted_test_msg.get());
    return extracted_test_msg;
}

std::shared_ptr<TestMsgT> DataStreamReader::readAtTimestamp(rcutils_time_point_value_t time) {
    openAndGetMessageType();
    auto bag_message = reader_.read_at_timestamp(time);
    auto extracted_test_msg = std::make_shared<TestMsgT>();
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
    serialization_.deserialize_message(&extracted_serialized_msg, extracted_test_msg.get());
    return extracted_test_msg;
}

std::shared_ptr<std::vector<std::shared_ptr<TestMsgT>>> DataStreamReader::readAtIndexRange(uint32_t index_begin, uint32_t index_end) {
    openAndGetMessageType();
    auto bag_message_vector = reader_.read_at_index_range(index_begin, index_end);
    // std::cout << "Number of messages: " << bag_message_vector->size() << std::endl;

    auto deserialized_bag_message_vector = std::make_shared<std::vector<std::shared_ptr<TestMsgT>>>();
    for (auto bag_message : *bag_message_vector) {
        auto extracted_test_msg = std::make_shared<TestMsgT>();
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization_.deserialize_message(&extracted_serialized_msg, extracted_test_msg.get());

        deserialized_bag_message_vector->push_back(extracted_test_msg); // ToDo: reserve the vector instead of pushing back
    }
    return deserialized_bag_message_vector;
}

std::shared_ptr<std::vector<std::shared_ptr<TestMsgT>>> DataStreamReader::readAtTimestampRange(rcutils_time_point_value_t time_begin, rcutils_time_point_value_t time_end) {
    openAndGetMessageType();
    auto bag_message_vector = reader_.read_at_timestamp_range(time_begin, time_end);
    // std::cout << "Number of messages: " << bag_message_vector->size() << std::endl;

    auto deserialized_bag_message_vector = std::make_shared<std::vector<std::shared_ptr<TestMsgT>>>();
    for (auto bag_message : *bag_message_vector) {
        auto extracted_test_msg = std::make_shared<TestMsgT>();
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization_.deserialize_message(&extracted_serialized_msg, extracted_test_msg.get());

        deserialized_bag_message_vector->push_back(extracted_test_msg); // ToDo: reserve the vector instead of pushing back
    }
    return deserialized_bag_message_vector;
}


}
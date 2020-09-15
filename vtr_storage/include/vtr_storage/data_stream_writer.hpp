#pragma once

#include <any>
#include <utility>

#include "vtr_storage/data_stream_base.hpp"
#include "vtr_storage/message.hpp"
#include "vtr_storage/sequential_append_writer.hpp"

namespace vtr {
namespace storage {

class DataStreamWriterBase : public DataStreamBase {
 public:
  DataStreamWriterBase(const std::string &data_directory_string,
                       const std::string &stream_name, bool append = false)
      : DataStreamBase(data_directory_string, stream_name), append_(append) {}
  ~DataStreamWriterBase(){};

  virtual void open() = 0;
  virtual void close() = 0;
  virtual int32_t write(const VTRMessage &anytype_message) = 0;

 protected:
  virtual rosbag2_storage::TopicMetadata createTopicMetadata() = 0;

  bool append_;
};

template <typename MessageType>
class DataStreamWriter : public DataStreamWriterBase {
 public:
  DataStreamWriter(const std::string &data_directory_string,
                   const std::string &stream_name, bool append = false);
  ~DataStreamWriter();

  void open() override;
  void close() override;

  // returns the id of the inserted message
  int32_t write(const VTRMessage &vtr_message) override;

 protected:
  rosbag2_storage::TopicMetadata createTopicMetadata() override;

  rclcpp::Serialization<MessageType> serialization_;
  rosbag2_storage::TopicMetadata tm_;
  std::shared_ptr<SequentialAppendWriter> writer_;
};
}  // namespace storage
}  // namespace vtr

#include "vtr_storage/data_stream_writer.inl"

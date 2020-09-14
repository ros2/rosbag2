#pragma once

#include <any>
#include <utility>

#include "vtr_storage/data_stream_base.hpp"
#include "vtr_storage/random_access_reader.hpp"

namespace vtr {
namespace storage {

class DataStreamReaderBase : public DataStreamBase {
 public:
  DataStreamReaderBase(const std::string &data_directory,
                       const std::string &stream_name)
      : DataStreamBase(data_directory, stream_name) {}
  ~DataStreamReaderBase() {}

  virtual void openAndGetMessageType() = 0;
  virtual void close() = 0;

  virtual std::shared_ptr<std::any> readAtIndex(int32_t index) = 0;
  virtual std::shared_ptr<std::any> readAtTimestamp(
      rcutils_time_point_value_t time) = 0;
  virtual std::shared_ptr<std::vector<std::shared_ptr<std::any>>>
  readAtIndexRange(int32_t index_begin, int32_t index_end) = 0;
  virtual std::shared_ptr<std::vector<std::shared_ptr<std::any>>>
  readAtTimestampRange(rcutils_time_point_value_t time_begin,
                       rcutils_time_point_value_t time_end) = 0;
};

template <typename MessageType>
class DataStreamReader : public DataStreamReaderBase {
 public:
  DataStreamReader(const std::string &data_directory,
                   const std::string &stream_name);
  ~DataStreamReader();

  void openAndGetMessageType() override;
  void close() override;

  std::shared_ptr<std::any> readAtIndex(int32_t index) override;
  std::shared_ptr<std::any> readAtTimestamp(
      rcutils_time_point_value_t time) override;
  std::shared_ptr<std::vector<std::shared_ptr<std::any>>> readAtIndexRange(
      int32_t index_begin, int32_t index_end) override;
  std::shared_ptr<std::vector<std::shared_ptr<std::any>>> readAtTimestampRange(
      rcutils_time_point_value_t time_begin,
      rcutils_time_point_value_t time_end) override;

  // next() isn't needed for now due to ranged random access. Can add in if
  // necessary

 protected:
  rclcpp::Serialization<MessageType> serialization_;
  std::shared_ptr<RandomAccessReader> reader_;
};

}  // namespace storage
}  // namespace vtr

#include "vtr_storage/data_stream_reader.inl"

#pragma once

#include <any>

#include "vtr_storage/data_bubble_base.hpp"
#include "vtr_storage/data_stream_reader.hpp"

namespace vtr {
namespace storage {

/// \brief DataBubble class. Container class for a specified range of messages
/// in a robochunk stream.
///        Allows for access and storage of messages in memory.
template <typename MessageType>
class DataBubble : public DataBubbleBase {
 public:
  DataBubble();

  ~DataBubble();

  /// \brief Initializes a data bubble with a data stream.
  /// \param A pointer to the associated data stream.
  void initialize(std::shared_ptr<DataStreamReaderBase> data_stream);

  /// \brief loads all of the messages associated with this bubble into memory.
  void load();

  /// \brief loads the specified message based on local index
  /// \details This is a local index (i.e. If the bubble wraps 20 messages then
  /// the local index is in the range (0-19))
  /// \param The local index.
  void load(int32_t local_idx);

  /// \brief loads the specified messages into memory based on an index range.
  /// \details This is a local index (i.e. If the bubble wraps 20 messages then
  /// the local index is in the range (0-19))
  /// \param The start index.
  /// \param The end index.
  void load(int32_t global_idx0, int32_t global_idx1);

  /// \brief loads a specific message based on a time tag into memory.
  /// \param time The time stamp of the message to be loaded.
  void load(TimeStamp time);

  /// \brief loads a range of messages based on time tags into memory.
  /// \param time Begining time stamp of the message to be loaded.
  /// \param time End time stamp of the message to be loaded.
  void load(TimeStamp time0, TimeStamp time1);

  /// \brief unloads all data associated with the vertex.
  void unload();

  /// \brief unloads A specific message.
  /// \param The index of the message to unload.
  void unload(int32_t local_idx);

  /// \brief unloads A range of messages
  /// \param The first index.
  /// \param The last index.
  void unload(int32_t local_idx0, int32_t local_idx1);

  /// \brief Inserts a message into the bubble.
  void insert(const std::any& message);

  /// \brief Gets the size of the bubble (number of messages)
  /// \return the size of the bubble.
  int32_t size();

  /// \brief Checks to see if a message is loaded based on index.
  /// \return true if the message is loaded, false otherwise.
  bool isLoaded(int32_t idx);

  /// \brief Checks to see if a message is loaded based on time.
  /// \return true if the message is loaded, false otherwise.
  bool isLoaded(TimeStamp time);

  /// \brief Retrieves a reference to the message.
  /// \param the index of the message.
  std::any& retrieve(int32_t local_idx);

  /// \brief Retrieves a reference to the message.
  /// \param The timestamp of the message.
  std::any& retrieve(TimeStamp time);

 private:
  /// \brief A pointer to the Robochunk stream.
  std::shared_ptr<DataStreamReader<MessageType>> data_stream_;
};
}  // namespace storage
}  // namespace vtr

#include "vtr_storage/data_bubble.inl"

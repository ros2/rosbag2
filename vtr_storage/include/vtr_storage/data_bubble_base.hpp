#pragma once

#include <any>
#include <map>

#include "rcutils/types.h"

#include "vtr_storage/data_stream_reader.hpp"

namespace vtr {
namespace storage {

using TimeStamp = rcutils_time_point_value_t;
using DataMap = std::map<int32_t, std::any>;

/// \brief Bubble indices into a robochunk stream.
struct ChunkIndices {
  /// \brief constructor
  ChunkIndices() {
    start_index = 0;
    stop_index = 0;
  }

  /// \brief Start index into the stream.
  int32_t start_index;
  /// \brief Stop index into the stream.
  int32_t stop_index;
  /// \brief Start time into the stream.
  TimeStamp start_time;
  /// \brief Stop time into the stream.
  TimeStamp stop_time;
};

/// \brief DataBubble class. Container class for a specified range of messages
/// in a robochunk stream.
///        Allows for access and storage of messages in memory.
class DataBubbleBase {
 public:
  DataBubbleBase();

  ~DataBubbleBase();

  /// \brief Sets the indicies for this bubble.
  /// \param The start index of this bubble.
  /// \param The end index of this bubble.
  bool setIndices(uint64_t index_begin, uint64_t index_end);

  /// \brief Sets the Time indices for this bubble.
  /// \param The start time of this bubble.
  /// \param The end time of this bubble.
  bool setTimeIndices(TimeStamp time_begin, TimeStamp time_end);

  /// \brief returns the number of bytes being used by this bubble.
  /// \param the number of bytes being used by this bubble.
  const uint64_t& memoryUsageBytes() { return memoryUsageBytes_; };

  /// \brief Initializes a data bubble with a data stream.
  /// \param A pointer to the associated data stream.
  virtual void initialize(
      std::shared_ptr<DataStreamReaderBase> data_stream) = 0;

  /// \brief loads all of the messages associated with this bubble into memory.
  virtual void load() = 0;

  /// \brief loads the specified message based on local index
  /// \details This is a local index (i.e. If the bubble wraps 20 messages then
  /// the local index is in the range (0-19))
  /// \param The local index.
  virtual void load(int32_t local_idx) = 0;

  /// \brief loads the specified messages into memory based on an index range.
  /// \details This is a local index (i.e. If the bubble wraps 20 messages then
  /// the local index is in the range (0-19))
  /// \param The start index.
  /// \param The end index.
  virtual void load(int32_t global_idx0, int32_t global_idx1) = 0;

  /// \brief loads a specific message based on a time tag into memory.
  /// \param time The time stamp of the message to be loaded.
  virtual void load(TimeStamp time) = 0;

  /// \brief loads a range of messages based on time tags into memory.
  /// \param time Begining time stamp of the message to be loaded.
  /// \param time End time stamp of the message to be loaded.
  virtual void load(TimeStamp time0, TimeStamp time1) = 0;

  /// \brief unloads all data associated with the vertex.
  virtual void unload() = 0;

  /// \brief unloads A specific message.
  /// \param The index of the message to unload.
  virtual void unload(int32_t local_idx) = 0;

  /// \brief unloads A range of messages
  /// \param The first index.
  /// \param The last index.
  virtual void unload(int32_t local_idx0, int32_t local_idx1) = 0;

  /// \brief Inserts a message into the bubble.
  virtual void insert(const std::any& message) = 0;

  /// \brief Gets the size of the bubble (number of messages)
  /// @return the size of the bubble.
  virtual int32_t size() = 0;

  /// \brief Checks to see if a message is loaded based on index.
  /// @return true if the message is loaded, false otherwise.
  virtual bool isLoaded(int32_t idx) = 0;

  /// \brief Checks to see if a message is loaded based on time.
  /// @return true if the message is loaded, false otherwise.
  virtual bool isLoaded(TimeStamp time) = 0;

  /// \brief Retrieves a reference to the message.
  /// \param the index of the message.
  virtual std::any& retrieve(int32_t local_idx) = 0;

  /// \brief Retrieves a reference to the message.
  /// \param The timestamp of the message.
  virtual std::any& retrieve(TimeStamp time) = 0;

  /// \brief provides an iterator to the begining of the bubble.
  /// @return Begin iterator into the bubble's data.
  DataMap::iterator begin() { return data_map_.begin(); }

  /// \brief provides an iterator to the end of the bubble.
  /// \brief Begin iterator into the bubble's data.
  DataMap::iterator end() { return data_map_.end(); }

 protected:
  /// \brief the current end index of the bubble.
  int32_t endIdx_;
  /// \brief The indices associated with this bubble.
  ChunkIndices indices_;
  /// \brief flag to determine if this bubble can be loaded by index.
  bool loadFromIndex_;
  /// \brief flag to determine if this bubble can be loaded by time.
  bool loadFromTime_;
  /// \brief The number of bytes being used by the bubble.
  uint64_t memoryUsageBytes_;
  /// \brief If the bubble has been loaded from disk. Defaults to false
  bool is_loaded_;
  /// \brief The map of currently loaded data.
  DataMap data_map_;
  /// \brief maps timestamps to indices in the data map.
  std::map<uint64_t, int32_t> time_map_;
};
}  // namespace storage
}  // namespace vtr

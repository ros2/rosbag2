/// @file DataBubble.hpp
/// @brief header file for the robochunk data bubble class.
/// @author Michael Paton, ASRL

#include "vtr_storage/DataBubbleBase.hpp"

namespace vtr {
namespace storage {

DataBubbleBase::DataBubbleBase()
    : endIdx_(0),
      loadFromIndex_(false),
      loadFromTime_(false),
      memoryUsageBytes_(0),
      is_loaded_(false) {}

DataBubbleBase::~DataBubbleBase() {}

bool DataBubbleBase::setIndices(uint64_t index_begin, uint64_t index_end) {
  if (index_end < index_begin) {
    // ToDo: include easylogging++
    // LOG(ERROR) << "ERROR: Invalid inex range (" << index_begin <<  ","  <<
    // index_end << ")";
    return false;
  }

  indices_.start_index = index_begin;
  indices_.stop_index = index_end;
  loadFromIndex_ = true;
  endIdx_ = (indices_.stop_index - indices_.start_index);
  if (endIdx_ == 0) {
    endIdx_ = -1;
  }
  return true;
}

bool DataBubbleBase::setTimeIndices(TimeStamp time_begin, TimeStamp time_end) {
  // if(time_end.nanoseconds_since_epoch() <
  // time_begin.nanoseconds_since_epoch()) {
  //     LOG(ERROR) << "ERROR: Invalid index range (" <<
  //     time_begin.nanoseconds_since_epoch() << "," <<
  //     time_end.nanoseconds_since_epoch() << ")"; return false;
  // }

  indices_.start_time = time_begin;
  indices_.stop_time = time_end;
  loadFromTime_ = true;
  return true;
}

}  // namespace storage
}  // namespace vtr

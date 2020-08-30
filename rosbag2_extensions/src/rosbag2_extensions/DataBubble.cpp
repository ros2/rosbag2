#include <rosbag2_extensions/DataBubble.hpp>
#include <iterator>

namespace rosbag2_extensions {

DataBubble::DataBubble() :
memoryUsageBytes_(0),
loadFromIndex_(false),
loadFromTime_(false),
endIdx(0),
is_loaded_(false) {

}

DataBubble::~DataBubble() {
    data_map_.clear();
    data_stream_.reset();
}

void DataBubble::initialize(std::shared_ptr<robochunk::base::ChunkStream> data_stream) {
    data_stream_ = data_stream;
}

uint32_t DataBubble::size() {
    return data_map_.size();
}

bool  DataBubble::isLoaded(uint32_t idx) {
    return data_map_.count(idx);
}

bool DataBubble::isLoaded(const robochunk::std_msgs::TimeStamp &time) {
  auto time_it = time_map_.find(time.nanoseconds_since_epoch()); // get the index from the time map
  return time_it != time_map_.end() && isLoaded(time_it->second);
}

void DataBubble::insert(Message &message) {
    data_map_.insert({endIdx,message});
    time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),endIdx});
    endIdx++;
    memoryUsageBytes_+=message.ByteSize();
}

bool DataBubble::setIndices(uint64_t index_begin,uint64_t index_end) {
    if(index_end < index_begin) {
        LOG(ERROR) << "ERROR: Invalid inex range (" << index_begin <<  ","  << index_end << ")";
        return false;
    }
    
    indices_.start_index = index0;
    indices_.stop_index = index1;
    loadFromIndex_ = true;
    endIdx = (indices_.stop_index-indices_.start_index);
    if(endIdx == 0) {endIdx = -1;}
    return true;
}

bool DataBubble::setTimeIndices(const robochunk::std_msgs::TimeStamp &time0,const robochunk::std_msgs::TimeStamp &time1) {
    if(time1.nanoseconds_since_epoch() < time0.nanoseconds_since_epoch()) {
        LOG(ERROR) << "ERROR: Invalid index range (" << time0.nanoseconds_since_epoch() << "," <<time1.nanoseconds_since_epoch() << ")";
        return false;
    }
    
    indices_.start_time = time0;
    indices_.stop_time = time1;
    loadFromTime_ = true;
}

void DataBubble::load() {
    if(is_loaded_ == true) {
      return;
    }
    // seek to the start idx
    if(loadFromIndex_ == true) {
        load(indices_.start_index,indices_.stop_index);
        is_loaded_ = true;
    } else if(loadFromTime_ == true) {
        load(indices_.start_time,indices_.stop_time);
        is_loaded_ = true;
    } else {
        is_loaded_ = false;
        //LOG(ERROR) << __func__ << "ERROR No indices provided, call setIndices, or setTimeIndices";
    }
}

void DataBubble::load(uint32_t idx) {
     if(data_stream_->seek(indices_.start_index+idx)) {
        data_map_.insert({idx,msgs::RobochunkMessage()});
        if(data_stream_->next(data_map_[idx]) == false) {
            data_map_.erase(idx);
            return;
        }
        
        auto stamp = data_map_[idx].baseMessage().header().sensor_time_stamp();
        time_map_.insert({stamp.nanoseconds_since_epoch(),idx});
        memoryUsageBytes_+= data_map_[idx].ByteSize();
    }
}

void DataBubble::load(uint32_t idx0,uint32_t idx1) {
    // Seek to the current data.
    if(data_stream_->seek(idx0)) {
        for(int idx = 0; idx <= idx1-idx0; ++idx) {
            DataMap::value_type map_insert(idx,msgs::RobochunkMessage());
            data_map_.insert(map_insert);
            if(data_stream_->next(data_map_[idx]) == false) {
                data_map_.erase(idx);
                return;
            }
            if(idx == 0){ indices_.start_time = data_map_[idx].header().sensor_time_stamp();}
            if(idx == idx1-idx0){ indices_.stop_time = data_map_[idx].header().sensor_time_stamp();}
            time_map_.insert({data_map_[idx].header().sensor_time_stamp().nanoseconds_since_epoch(),idx});
            memoryUsageBytes_+= data_map_[idx].ByteSize();
        }
    }
}

void DataBubble::load(std_msgs::TimeStamp time) {
  /// Seek in the stream to this time.
  if(data_stream_->seek(time)) {
    Message message; 
    if(data_stream_->next(message) == true) {
      // determine the appropriate index.
      auto seq_id = message.header().sequence_id(); 
      if(loadFromIndex_ == true && (seq_id < indices_.start_index || seq_id > indices_.stop_index)) {
        LOG(DEBUG) << "The seek timestamp for the vertex is inconsistent with this bubble''s indices.";
        LOG(DEBUG) << "seek time: " << time.nanoseconds_since_epoch() << " msg time: " << message.header().sensor_time_stamp().nanoseconds_since_epoch();
        LOG(DEBUG) << " msg seq_id: " << seq_id << " bubble indices: (" << indices_.start_index << "," << indices_.stop_index << ")";
        // uh oh, we might have grabbed the wrong message. try to seek from index.
        message.Clear();
        // if this is there is only one message in the bubble, then grab it using the index.
        if(indices_.start_index == indices_.stop_index) {
          LOG(DEBUG) << "Attempting to seek from index...";
          if(data_stream_->seek(indices_.start_index) && data_stream_->next(message)) {
            auto seq_id = message.header().sequence_id(); 
            LOG(DEBUG) << "seek time: " << time.nanoseconds_since_epoch() << " msg time: " << message.header().sensor_time_stamp().nanoseconds_since_epoch();
            LOG(DEBUG) << " msg seq_id: " << seq_id << " bubble indices: (" << indices_.start_index << "," << indices_.stop_index << ")";
            auto index = seq_id - indices_.start_index;
            LOG(DEBUG) << "success! ";
            data_map_[index] = message;
            time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),index});
            return;
          }
        }
        throw std::runtime_error("Timestaps are inconsistent with Indices in the stream data!");
      }
      auto index = seq_id - indices_.start_index;
      data_map_[index] = message;
      time_map_.insert({message.header().sensor_time_stamp().nanoseconds_since_epoch(),index});
    }
  } else {
    LOG(DEBUG) << __func__ << "Boom! we failed to seek!";
    throw std::runtime_error("Boom! we failed to seek!");
  }
}

void DataBubble::load(std_msgs::TimeStamp time0,std_msgs::TimeStamp time1) {
   // TODO: How to do?
    bool continue_load = true;
    if(data_stream_->seek(time0)) {
        while(continue_load) {
            endIdx++;
            data_map_[endIdx] = msgs::RobochunkMessage();
            if(data_stream_->next(data_map_[endIdx]) == false || data_map_[endIdx].header().sensor_time_stamp().nanoseconds_since_epoch() > time1.nanoseconds_since_epoch()) {
                data_map_.erase(endIdx);
                endIdx--;
                indices_.stop_index = size()-1;
                return;
            }
            auto stamp = data_map_[endIdx].baseMessage().header().sensor_time_stamp();
            time_map_.insert({stamp.nanoseconds_since_epoch(),endIdx});
            if(indices_.start_index == 0) {
                indices_.start_index = data_map_[endIdx].baseMessage().header().sequence_id();
            }
        }
    }
    else {
        LOG(ERROR) << "seek falied!";
    }
}

void DataBubble::unload() {
    is_loaded_ = false;
    memoryUsageBytes_ = 0;
    data_map_.clear();
}

void DataBubble::unload(uint32_t idx) {
    memoryUsageBytes_ -= data_map_[idx].ByteSize();
    data_map_.erase(idx);
}

void DataBubble::unload(uint32_t idx0,uint32_t idx1) {
  for(int index = idx0; index < idx1; ++index) {
    unload(index);
  }
}

Message &DataBubble::retrieve(uint32_t idx) {
    if(isLoaded(idx) == false) {
        load(idx);
    }
    if (isLoaded(idx) == false) {
      throw std::out_of_range("DataBubble has not data at this index.");
    }

    return data_map_[idx];
}

Message& DataBubble::retrieve(const std_msgs::TimeStamp &time) {
    // check to see if its in the time map
    auto stamp_nanoseconds = time.nanoseconds_since_epoch();
    if(time_map_.find(stamp_nanoseconds) != std::end(time_map_)) {
        // hooray we have a valid idx
        return retrieve(time_map_[stamp_nanoseconds]);
    }

    // The message is not in the bubble's memory, try to load it.
    if(loadFromIndex_ == true || loadFromTime_ == true) {
        // Load from disk.
        load(time);

        // If we successfully loaded, then return the data
        if(time_map_.find(stamp_nanoseconds) != std::end(time_map_)) {
          return retrieve(time_map_[stamp_nanoseconds]);
        }
    }
    // otherwise throw.
    throw std::out_of_range("DataBubble has no data at this time stamp.");
}

}

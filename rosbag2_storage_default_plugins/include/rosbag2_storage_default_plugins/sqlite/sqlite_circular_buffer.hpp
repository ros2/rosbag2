#ifndef ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_CIRCULAR_BUFFER_HPP_
#define ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_CIRCULAR_BUFFER_HPP_

#include <stdexcept>
#include <vector>

#include "rosbag2_storage_default_plugins/visibility_control.hpp"

namespace rosbag2_storage_plugins
{

template < typename T >
class ROSBAG2_STORAGE_DEFAULT_PLUGINS_PUBLIC CircularBuffer
{
public:
  CircularBuffer(const int size) {
    if (size <= 0) {
      throw std::invalid_argument("Invalid circular buffer size.");
    }
    size_ = size;
    array_.resize(size_);
  }

  unsigned int size() {
    return size_;
  }

  unsigned int failed_counter() {
    return failed_counter_;
  }

  unsigned int elements_num() {
    return elements_num_;
  }

  void enqueue(T item) {
    if (is_full()) {
      ++failed_counter_;
      return;
    }
    array_[this->rear_index_] = item;
    this->rear_index_ = (this->rear_index_ + 1) % size_;
    elements_num_++;
  }

  void dequeue() {
    if (is_empty()) {
      return;
    }
    front_index_ = (front_index_ + 1) % size_;
    elements_num_--;
  }

  T front(){
    return is_empty() ? nullptr : array_[front_index_];
  }

  bool is_empty() {
    return rear_index_ == front_index_;
  }

  bool is_full() {
    return (rear_index_ + 1) % size_ == front_index_;
  }

private:
  std::vector<T> array_;
  int front_index_ = 0, rear_index_ = 0, size_ = 0, failed_counter_ = 0, elements_num_ = 0;
};
}

#endif // ROSBAG2_STORAGE_DEFAULT_PLUGINS__SQLITE__SQLITE_CIRCULAR_BUFFER_HPP_

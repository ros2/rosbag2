#ifndef ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_
#define ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/visibility_control.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"

namespace rosbag2_cpp
{
namespace cache
{

class ROSBAG2_CPP_PUBLIC TransientLocalMessagesCache
{
public:
  using MessageSharedPtr = std::shared_ptr<const rosbag2_storage::SerializedBagMessage>;

  void add_topic(const std::string & topic_name, size_t queue_depth);
  void remove_topic(const std::string & topic_name);
  bool has_topic(const std::string & topic_name) const;

  void push(const std::string & topic_name, MessageSharedPtr message);

  std::vector<MessageSharedPtr> get_messages_sorted_by_timestamp() const;

  void clear();
  size_t size() const;

private:
  struct TopicQueue
  {
    size_t max_depth{0};
    std::deque<MessageSharedPtr> messages;
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicQueue> topic_queues_;
};

}  // namespace cache
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__CACHE__TRANSIENT_LOCAL_MESSAGES_CACHE_HPP_

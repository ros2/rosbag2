#include "rosbag2_transport/rosbag2_transport.hpp"

namespace rosbag2_transport
{

void Rosbag2Transport::record(const std::vector<std::string> & topic_names)
{
  for (auto topic : topic_names) {
    fprintf(stderr, "going to record %s\n", topic.c_str());
  }
}

}  // namespace rosbag2_transport

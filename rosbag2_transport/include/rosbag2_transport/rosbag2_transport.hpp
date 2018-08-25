#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include <string>
#include <vector>

#include "rosbag2_transport/visibility_control.h"

namespace rosbag2_transport
{

class Rosbag2Transport
{
public:
  Rosbag2Transport();

  virtual ~Rosbag2Transport();

  void record(const std::vector<std::string> & topic_names)
  {
    for (auto topic : topic_names) {
      fprintf(stderr, "going to record %s\n", topic.c_str());
    }
  }
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

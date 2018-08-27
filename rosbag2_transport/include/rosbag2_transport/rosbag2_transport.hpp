#ifndef ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_
#define ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include <string>
#include <vector>

#include "rosbag2_transport/visibility_control.h"

namespace rosbag2_transport
{

class ROSBAG2_TRANSPORT_PUBLIC Rosbag2Transport
{
public:
  Rosbag2Transport() = default;

  virtual ~Rosbag2Transport() = default;

  void record(const std::vector<std::string> & topic_names);
};

}  // namespace rosbag2_transport

#endif  // ROSBAG2_TRANSPORT__ROSBAG2_TRANSPORT_HPP_

#include "qos.hpp"

namespace rosbag2_transport
{

std::ostream& operator<<(std::ostream& os, rmw_time_t time)
{
  os.precision(3);
  double ms = (time.sec * 1000L) + (time.nsec / 1000000.0);
  os << ms << " ms";
  return os;
}

bool operator==(const rmw_time_t & left, const rmw_time_t & right)
{
  return left.sec == right.sec && left.nsec == right.nsec;
}

std::ostream& operator<<(std::ostream& os, const rclcpp::QoS& qos)
{
  const auto & p = qos.get_rmw_qos_profile();
  os << "History: " << History::_from_integral(p.history) << " (" << p.depth << ")" << std::endl;
  os << "Reliability: " << Reliability::_from_integral(p.reliability) << std::endl;
  os << "Durability: " << Durability::_from_integral(p.durability) << std::endl;
  os << "Deadline: " << p.deadline << std::endl;
  os << "Lifespan: " << p.lifespan << std::endl;
  os << "Liveliness: " << Liveliness::_from_integral(p.liveliness)
     << " (" << p.liveliness_lease_duration << ")" << std::endl;
  return os;
}

bool operator==(const rclcpp::QoS& left, const rclcpp::QoS & right)
{
  const auto & pl = left.get_rmw_qos_profile();
  const auto & pr = right.get_rmw_qos_profile();
  return pl.history == pr.history &&
         pl.depth == pr.depth &&
         pl.reliability == pr.reliability &&
         pl.durability == pr.durability &&
         pl.deadline == pr.deadline &&
         pl.lifespan == pr.lifespan &&
         pl.liveliness == pr.liveliness &&
         pl.liveliness_lease_duration == pr.liveliness_lease_duration;
}

}

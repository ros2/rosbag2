#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;  // NOLINT

struct Point
{
  float x, y, z;
  Point(float x, float y, float z)
      : x(x), y(y), z(z) {}
};

sensor_msgs::msg::PointCloud2::SharedPtr createRandomPointcloud2(size_t num_points)
{
  std::vector<Point> points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    points.emplace_back(std::rand() % 255, std::rand() % 255, std::rand() % 255);
  }

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header = std_msgs::msg::Header();
  cloud->header.frame_id = "pointcloud_frame";
  cloud->header.stamp = rclcpp::Clock().now();

  cloud->is_bigendian = false;
  cloud->is_dense = true;

  cloud->height = 1;
  cloud->width = (uint32_t) points.size();

  cloud->fields.resize(3);
  cloud->fields[0].name = "x";
  cloud->fields[1].name = "y";
  cloud->fields[2].name = "z";

  sensor_msgs::msg::PointField::_offset_type offset = 0;
  for (uint32_t i = 0; i < cloud->fields.size(); ++i, offset += sizeof(float)) {
    cloud->fields[i].count = 1;
    cloud->fields[i].offset = offset;
    cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud->point_step = offset;
  cloud->row_step = cloud->point_step * cloud->width;
  cloud->data.resize(cloud->row_step * cloud->height);

  auto floatData = reinterpret_cast<float *>(cloud->data.data());
  for (uint32_t i = 0; i < cloud->width; ++i) {
    floatData[i * (cloud->point_step / sizeof(float)) + 0] = points[i].x;
    floatData[i * (cloud->point_step / sizeof(float)) + 1] = points[i].y;
    floatData[i * (cloud->point_step / sizeof(float)) + 2] = points[i].z;
  }

  return cloud;
}

namespace nodes
{

class PointCloud2Publisher : public rclcpp::Node {
public:
  PointCloud2Publisher(const std::string & name, const std::string & topic)
      : Node(name)
  {
    this->declare_parameter("frequency");
    this->declare_parameter("max_count");
    this->declare_parameter("size");
    this->declare_parameter("delay");
    this->declare_parameter("benchmark_path");

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    frequency = parameters_client->get_parameter<uint32_t>("frequency", 100);
    if(frequency == 0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency can't be 0. Exiting.");
      rclcpp::shutdown(nullptr, "frequency error");
    }
    max_count = parameters_client->get_parameter<uint32_t>("max_count", 100);
    size = parameters_client->get_parameter<uint32_t>("size", 100000);
    delay = parameters_client->get_parameter<uint32_t>("delay", 0);
    benchmark_path = parameters_client->get_parameter<std::string>("benchmark_path", "");

    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
    delay_timer = this->create_wall_timer(std::chrono::milliseconds(delay), std::bind(&PointCloud2Publisher::delay_callback, this));
    random_pointcloud2 = createRandomPointcloud2(size);
  }

private:
  void delay_callback()
  {
    std::cout << this->get_name() <<  ": Delay finished" << std::endl;
    timer = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&PointCloud2Publisher::timer_callback, this));
    delay_timer->cancel();
  }

  void timer_callback()
  {
    uint32_t static current_msg_count = 0;

    publisher->publish(*random_pointcloud2);

    std::cout << this->get_name() << ": " << current_msg_count << std::endl;
    if(++current_msg_count == max_count) {
      timer->cancel();
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<sensor_msgs::msg::PointCloud2> random_pointcloud2;
  std::string benchmark_path;
  uint32_t frequency;
  uint32_t max_count;
  uint32_t size;
  uint32_t delay;
  rclcpp::TimerBase::SharedPtr delay_timer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

}  // namespace nodes

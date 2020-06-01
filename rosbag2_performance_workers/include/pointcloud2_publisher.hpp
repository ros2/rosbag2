// Copyright 2019, Martin Idel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

sensor_msgs::msg::PointCloud2::SharedPtr createPointCloud2WithRandomPoints(
    size_t num_points)
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
  PointCloud2Publisher(const std::string & node_name, const std::string & topic_name)
      : Node(node_name)
  {
    publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
    timer = this->create_wall_timer(100ms, std::bind(&PointCloud2Publisher::timer_callback, this));
    random_pointcloud2_ = createPointCloud2WithRandomPoints(100000);
  }

private:
  void timer_callback()
  {
    publisher->publish(*random_pointcloud2_);
  }

  std::shared_ptr<sensor_msgs::msg::PointCloud2> random_pointcloud2_;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
};

}  // namespace nodes

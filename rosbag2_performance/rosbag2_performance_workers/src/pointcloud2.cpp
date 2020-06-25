// Copyright 2020, Robotec.ai sp. z o.o.
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

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

#include "worker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloud2Publisher : public Worker<sensor_msgs::msg::PointCloud2>
{
public:
  explicit PointCloud2Publisher(const std::string & name)
  : Worker<sensor_msgs::msg::PointCloud2>(name)
  {
    message = createRandomPointcloud2(size);
  }

  sensor_msgs::msg::PointCloud2 getMessage(const uint32_t & size) final
  {
    (void)size;  // supress unused
    return message;
  }

private:
  struct Point
  {
    float x, y, z;
    Point(float x, float y, float z)
    : x(x), y(y), z(z) {}
  };

  sensor_msgs::msg::PointCloud2 createRandomPointcloud2(size_t num_points)
  {
    std::vector<Point> points;
    points.reserve(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      points.emplace_back(std::rand() % 255, std::rand() % 255, std::rand() % 255);
    }

    auto pc2c = sensor_msgs::msg::PointCloud2();
    pc2c.header = std_msgs::msg::Header();
    pc2c.header.frame_id = "worker";
    pc2c.header.stamp = rclcpp::Clock().now();

    pc2c.is_bigendian = false;
    pc2c.is_dense = true;

    pc2c.height = 1;
    pc2c.width = (uint32_t)points.size();

    pc2c.fields.resize(3);
    pc2c.fields[0].name = "x";
    pc2c.fields[1].name = "y";
    pc2c.fields[2].name = "z";

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    for (uint32_t i = 0; i < pc2c.fields.size(); ++i, offset += sizeof(float)) {
      pc2c.fields[i].count = 1;
      pc2c.fields[i].offset = offset;
      pc2c.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    pc2c.point_step = offset;
    pc2c.row_step = pc2c.point_step * pc2c.width;
    pc2c.data.resize(pc2c.row_step * pc2c.height);

    auto floatData = reinterpret_cast<float *>(pc2c.data.data());
    for (uint32_t i = 0; i < pc2c.width; ++i) {
      floatData[i * (pc2c.point_step / sizeof(float)) + 0] = points[i].x;
      floatData[i * (pc2c.point_step / sizeof(float)) + 1] = points[i].y;
      floatData[i * (pc2c.point_step / sizeof(float)) + 2] = points[i].z;
    }

    return pc2c;
  }

  sensor_msgs::msg::PointCloud2 message;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Context ctx;
  auto node = std::make_shared<PointCloud2Publisher>("pointcloud2_publisher");
  if (rclcpp::ok()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return rclcpp::contexts::get_global_default_context()->shutdown_reason()
    .compare("frequency error") == 0;
}

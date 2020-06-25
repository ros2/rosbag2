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
#include "std_msgs/msg/byte_multi_array.hpp"

class ByteArrayPublisher : public Worker<std_msgs::msg::ByteMultiArray>
{
public:
  explicit ByteArrayPublisher(const std::string & name)
  : Worker<std_msgs::msg::ByteMultiArray>(name)
  {
    message.data = randomByteArrayData(size);
  }

  std_msgs::msg::ByteMultiArray getMessage(const uint32_t & size) final
  {
    (void)size;  // supress unused
    return message;
  }

private:
  std::vector<uint8_t> randomByteArrayData(size_t size)
  {
    std::vector<uint8_t> byte(size, 0);

    for (size_t i = 0; i < size; ++i) {
      byte[i] = std::rand() % 255;
    }
    return byte;
  }

  std_msgs::msg::ByteMultiArray message;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Context ctx;
  auto node = std::make_shared<ByteArrayPublisher>("bytearray_publisher");
  if (rclcpp::ok()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return rclcpp::contexts::get_global_default_context()->shutdown_reason()
    .compare("frequency error") == 0;
}

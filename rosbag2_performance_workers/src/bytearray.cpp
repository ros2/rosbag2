#include <string>
#include <vector>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

#include "worker.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

class ByteArrayPublisher : public Worker<std_msgs::msg::ByteMultiArray> {
public:
  ByteArrayPublisher(const std::string & name)
  : Worker<std_msgs::msg::ByteMultiArray>(name)
  {
    message.data = randomByteArrayData(size);
  }

  std_msgs::msg::ByteMultiArray getMessage(const uint32_t &size) final {
    (void)size; // supress unused
    return message;
  }
private:

  std::vector<uint8_t> randomByteArrayData(size_t size) {
    std::vector<uint8_t> byte(size, 0);

    for (size_t i = 0; i < size; ++i) {
      byte[i] = std::rand() % 255;
    }
    return byte;
  }

  std_msgs::msg::ByteMultiArray message;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Context ctx;
  auto node = std::make_shared<ByteArrayPublisher>("bytearray_publisher");
  if(rclcpp::ok()) {
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  return (rclcpp::contexts::get_global_default_context()->shutdown_reason().compare("frequency error") == 0);
}

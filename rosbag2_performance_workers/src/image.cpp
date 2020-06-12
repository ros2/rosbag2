#include <string>
#include <vector>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

#include "worker.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImagePublisher : public Worker<sensor_msgs::msg::Image>
{
public:
    ImagePublisher(const std::string &name)
        : Worker<sensor_msgs::msg::Image>(name)
    {
        message.header = std_msgs::msg::Header();
        message.header.stamp = rclcpp::Clock().now();

        message.encoding = "rgba8";
        message.height = static_cast<sensor_msgs::msg::Image::_height_type>(size);
        message.width = static_cast<sensor_msgs::msg::Image::_width_type>(size);
        message.step = static_cast<sensor_msgs::msg::Image::_step_type>(size * 4);
        message.data = randomImageData(size * 4 * size);
    }

    sensor_msgs::msg::Image getMessage(const uint32_t &size) final {
        (void)size; // supress unused
        return message;
    }

private:
    std::vector<uint8_t> randomImageData(size_t size)
    {
        std::vector<uint8_t> pixels(size, 0);

        for (size_t i = 0; i < size; ++i)
        {
            pixels[i] = std::rand() % 255;
        }
        return pixels;
    }

    sensor_msgs::msg::Image message;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Context ctx;
    auto node = std::make_shared<ImagePublisher>("image_publisher");
    if (rclcpp::ok())
    {
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    return (rclcpp::contexts::get_global_default_context()->shutdown_reason().compare("frequency error") == 0);
}

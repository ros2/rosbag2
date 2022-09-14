
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

class ThroughputTester : public rclcpp::Node
{
public:
  explicit ThroughputTester() : rclcpp::Node("throughput_tester")
  {

  }

  void run_test()
  {
    RCLCPP_INFO(get_logger(), "Test start");
    write_results();
    RCLCPP_INFO(get_logger(), "Test complete");
  }

  void write_results()
  {
  }

  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ThroughputTester>();
  executor.add_node(node);

  // The benchmark has its own control loop but uses spinning for parameters
  std::thread spin_thread([&executor]() {executor.spin();});

  node->run_test();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

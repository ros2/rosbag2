
#include "rclcpp/rclcpp.hpp"

class ThroughputTester : public rclcpp::Node
{
public:
  explicit ThroughputTester() : rclcpp::Node("throughput_tester")
  {

  }

  void start()
  {


    write_results();
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

  bench->start_benchmark();
  RCLCPP_INFO(bench->get_logger(), "Benchmark terminated");
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

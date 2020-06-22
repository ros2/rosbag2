#include "writer_benchmark.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Context ctx;
  auto bench = std::make_shared<WriterBenchmark>();
  bench->startBenchmark();
  RCLCPP_INFO(bench->get_logger(), "Benchmark terminated");
  return 0;
}

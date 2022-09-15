#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"


#include "rosbag2_storage/yaml.hpp"

namespace fs = std::filesystem;

struct StorageConfig {
  std::string storage_id;
  std::string storage_config_file;
};

const std::vector<StorageConfig> storage_configs = {
  {"mcap", "mcap_basic.yaml"},
  {"sqlite3", "sqlite_resilient.yaml"},
  {"sqlite3", "sqlite_optimized.yaml"},
};

struct ProducerConfig {
  int num_topics;
  int size_per_message;
  int messages_per_topic;
};

const std::vector<ProducerConfig> producers {
  {0, 0, 0},
  {1, 100, 1000},
  {1, 1000, 1000},
  {1, 100000, 1000},
  {10, 1000, 1000},
  {100, 1000, 100},
  {1000, 1000, 10},
};

struct TestResult {
  std::string test_name;
  int total_size;
  long time;
};

class ThroughputTester : public rclcpp::Node
{
public:
  explicit ThroughputTester() : rclcpp::Node("throughput_tester")
  {
  }

  std_msgs::msg::ByteMultiArray generate_msg(size_t message_size)
  {
    std_msgs::msg::ByteMultiArray message;
    message.data.reserve(message_size);
    for (auto i = 0u; i < message_size; ++i) {
      message.data.emplace_back(std::rand() % 255);
    }
    return message;
  }

  void run_test(
    const std::string & base_dir,
    const std::string & storage_id,
    const std::string & storage_config_file,
    int messages_per_topic,
    int num_topics,
    int size_per_message)
  {
    using clock_type = std::chrono::steady_clock;
    auto t_start = clock_type::now();
    using Msg = std_msgs::msg::ByteMultiArray;
    std::stringstream run_name_s;
    run_name_s << base_dir << "/"
      << "run_" << storage_id << "_" << storage_config_file << "_"
      << num_topics << "topics_"
      << messages_per_topic << "msgs_"
      << size_per_message << "permsg";
    std::string run_name = run_name_s.str();
    RCLCPP_INFO_STREAM(get_logger(), "Test start " << run_name);

    std::map<std::string, Msg> msgs;
    for (int itop = 0; itop < num_topics; itop++) {
      std::string topic_name = "testtopic" + std::to_string(itop);
      msgs[topic_name] = generate_msg(size_per_message);
    }

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = run_name;
    storage_opts.storage_id = storage_id;
    storage_opts.storage_config_uri = storage_config_file;
    rosbag2_cpp::ConverterOptions converter_opts;

    rclcpp::Time timestamp{0};
    rclcpp::Duration dt{0, 10000};
    {
      auto writer = rosbag2_cpp::Writer();
      writer.open(storage_opts, converter_opts);

      for (int imsg = 0; imsg < messages_per_topic; imsg++) {
        for (auto tm : msgs) {
          writer.write(tm.second, tm.first, timestamp);
          timestamp += dt;
        }
      }
    }

    fs::path dir{run_name};
    int total_size = 0;
    for (auto const& dir_entry : fs::directory_iterator{dir})
    {
      auto p = dir_entry.path();
      if (p.filename() == "metadata.yaml") {
      } else {
        total_size += fs::file_size(p);
        fs::remove(p);
      }
    }
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock_type::now() - t_start).count();

    results_.push_back({run_name, total_size, duration});
    RCLCPP_INFO(get_logger(), "Test complete");
  }

  void run_tests()
  {
    std::string base_dir = "test_run" + std::to_string(get_clock()->now().nanoseconds());
    for (auto producer : producers) {
      for (auto storage_config : storage_configs) {
        run_test(
          base_dir,
          storage_config.storage_id, storage_config.storage_config_file,
          producer.messages_per_topic, producer.num_topics, producer.size_per_message);
      }
    }
    write_results();
  }

  void write_results()
  {
    std::ofstream ofs{"results.csv"};
    const auto delim = ',';
    ofs << "run_name" << delim;
    ofs << "total_size" << delim;
    ofs << "time_ms";
    ofs << std::endl;

    for (const auto & result : results_) {
      ofs << result.test_name << delim;
      ofs << result.total_size << delim;
      ofs << result.time;
      ofs << std::endl;
    }
  }

  std::vector<TestResult> results_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ThroughputTester>();
  executor.add_node(node);

  std::thread spin_thread([&executor]() {executor.spin();});

  node->run_tests();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

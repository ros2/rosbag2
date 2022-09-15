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
};

struct ProducerConfig {
  int num_topics;
  int size_per_message;
  int messages_per_topic;
};

const std::vector<ProducerConfig> producers {
  {0, 0, 0},
  // Single Topic
  {1, 100, 1000},
  {1, 1000, 1000},
  {1, 100000, 1000},
  {1, 1000000, 1000},
  // 100 MB variously
  {10, 1000, 10000},
  {100, 1000, 1000},
  {1000, 1000, 100},
  // 1 GB variously
  {10, 10000, 10000},
  {100, 10000, 1000},
  {1000, 10000, 100},
  // 5 GB variously
  {1, 10000000, 500},
  {10, 1000000, 500},
  {100, 1000000, 50},
};

struct TestResult {
  StorageConfig storage;
  ProducerConfig producer;
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
    StorageConfig storage,
    ProducerConfig producer)
  {
    using clock_type = std::chrono::steady_clock;
    auto t_start = clock_type::now();
    using Msg = std_msgs::msg::ByteMultiArray;
    std::stringstream run_name_s;
    run_name_s
      << "run_" << storage.storage_id << "_" << storage.storage_config_file << "_"
      << producer.num_topics << "topics_"
      << producer.messages_per_topic << "msgs_"
      << producer.size_per_message << "permsg";
    std::string run_name = run_name_s.str();
    RCLCPP_INFO_STREAM(get_logger(), "Test start " << run_name);

    std::map<std::string, Msg> msgs;
    for (int itop = 0; itop < producer.num_topics; itop++) {
      std::string topic_name = "testtopic" + std::to_string(itop);
      msgs[topic_name] = generate_msg(producer.size_per_message);
    }

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = base_dir + "/" + run_name;
    storage_opts.storage_id = storage.storage_id;
    storage_opts.storage_config_uri = storage.storage_config_file;
    rosbag2_cpp::ConverterOptions converter_opts;

    rclcpp::Time timestamp{0};
    rclcpp::Duration dt{0, 10000};
    {
      auto writer = rosbag2_cpp::Writer();
      writer.open(storage_opts, converter_opts);

      for (int imsg = 0; imsg < producer.messages_per_topic; imsg++) {
        for (auto tm : msgs) {
          writer.write(tm.second, tm.first, timestamp);
          timestamp += dt;
        }
      }
    }

    int total_size = 0;
    for (auto const& dir_entry : fs::directory_iterator{storage_opts.uri})
    {
      auto p = dir_entry.path();
      if (p.filename() == "metadata.yaml") {
      } else {
        total_size += fs::file_size(p);
        fs::remove(p);
      }
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(clock_type::now() - t_start).count();
    results_.push_back({storage, producer, total_size, duration});
    RCLCPP_INFO_STREAM(get_logger(), "Test complete in " << duration << "ms");
  }

  void run_tests()
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::stringstream ts;
    ts << "test_run_" << std::put_time(&tm, "%Y-%m-%d-%H%M%S");
    std::string base_dir = ts.str();

    int total_tests = storage_configs.size() * producers.size();
    int current_test = 1;

    for (auto producer : producers) {
      if (!rclcpp::ok()) break;
      for (auto storage_config : storage_configs) {
        if (!rclcpp::ok()) break;
        RCLCPP_INFO_STREAM(get_logger(), "TEST " << current_test << "/" << total_tests);
        run_test(base_dir, storage_config, producer);
        current_test++;
      }
    }
    write_results(base_dir);
  }

  void write_results(const std::string & base_dir)
  {
    auto result_path = base_dir + "/results.csv";
    std::ofstream ofs{result_path};
    const auto delim = ',';
    ofs << "storage" << delim;
    ofs << "config" << delim;
    ofs << "max_cache_size" << delim;
    ofs << "num_topics" << delim;
    ofs << "size_per_msg" << delim;
    ofs << "msgs_per_topic" << delim;
    ofs << "total_size" << delim;
    ofs << "time_ms";
    ofs << std::endl;

    for (const auto & result : results_) {
      ofs << result.storage.storage_id << delim;
      ofs << result.storage.storage_config_file << delim;
      ofs << 0 << delim;
      ofs << result.producer.num_topics << delim;
      ofs << result.producer.size_per_message << delim;
      ofs << result.producer.messages_per_topic << delim;
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

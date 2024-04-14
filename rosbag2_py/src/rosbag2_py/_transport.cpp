// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <csignal>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/yaml.hpp"
#include "rosbag2_transport/bag_rewrite.hpp"
#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/player.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/recorder.hpp"

#include "./pybind11.hpp"

namespace py = pybind11;
typedef std::unordered_map<std::string, rclcpp::QoS> QoSMap;

namespace
{

rclcpp::QoS qos_from_handle(const py::handle source)
{
  PyObject * raw_obj = PyObject_CallMethod(source.ptr(), "get_c_qos_profile", "");
  const auto py_obj = py::cast<py::object>(raw_obj);
  const auto rmw_qos_profile = py_obj.cast<rmw_qos_profile_t>();
  const auto qos_init = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile);
  return rclcpp::QoS{qos_init, rmw_qos_profile};
}

QoSMap qos_map_from_py_dict(const py::dict & dict)
{
  QoSMap value;
  for (const auto & item : dict) {
    auto key = std::string(py::str(item.first));
    value.insert({key, qos_from_handle(item.second)});
  }
  return value;
}

/**
 * Simple wrapper subclass to provide nontrivial type conversions for python properties.
 */
template<class T>
struct OptionsWrapper : public T
{
public:
  void setDelay(double delay)
  {
    this->delay = rclcpp::Duration::from_nanoseconds(
      static_cast<rcl_duration_value_t>(RCUTILS_S_TO_NS(delay)));
  }

  double getPlaybackDuration() const
  {
    return RCUTILS_NS_TO_S(static_cast<double>(this->playback_duration.nanoseconds()));
  }

  void setPlaybackDuration(double playback_duration)
  {
    this->playback_duration = rclcpp::Duration::from_nanoseconds(
      static_cast<rcl_duration_value_t>(RCUTILS_S_TO_NS(playback_duration)));
  }

  double getDelay() const
  {
    return RCUTILS_NS_TO_S(static_cast<double>(this->delay.nanoseconds()));
  }

  void setStartOffset(double start_offset)
  {
    this->start_offset = static_cast<rcutils_time_point_value_t>(RCUTILS_S_TO_NS(start_offset));
  }

  double getStartOffset() const
  {
    return RCUTILS_NS_TO_S(static_cast<double>(this->start_offset));
  }

  void setPlaybackUntilTimestamp(int64_t playback_until_timestamp)
  {
    this->playback_until_timestamp =
      static_cast<rcutils_time_point_value_t>(playback_until_timestamp);
  }

  int64_t getPlaybackUntilTimestamp() const
  {
    return this->playback_until_timestamp;
  }

  void setTopicQoSProfileOverrides(const py::dict & overrides)
  {
    py_dict = overrides;
    this->topic_qos_profile_overrides = qos_map_from_py_dict(overrides);
  }

  const py::dict & getTopicQoSProfileOverrides() const
  {
    return py_dict;
  }

  py::dict py_dict;
};
typedef OptionsWrapper<rosbag2_transport::PlayOptions> PlayOptions;
typedef OptionsWrapper<rosbag2_transport::RecordOptions> RecordOptions;

}  // namespace

namespace rosbag2_py
{

class Player
{
public:
  using SignalHandlerType = void (*)(int);

  Player()
  {
    rclcpp::init(0, nullptr);
  }

  virtual ~Player()
  {
    rclcpp::shutdown();
  }

  static void cancel()
  {
    exit_ = true;
    wait_for_exit_cv_.notify_all();
  }

  void play(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options)
  {
    play_impl(storage_options, play_options, false);
  }

  void burst(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options,
    size_t num_messages)
  {
    play_impl(storage_options, play_options, true, num_messages);
  }

protected:
  static void signal_handler(int sig_num)
  {
    if (sig_num == SIGINT || sig_num == SIGTERM) {
      deferred_sig_number_ = sig_num;
      rosbag2_py::Player::cancel();
    }
  }

  static void install_signal_handlers()
  {
    deferred_sig_number_ = -1;
    old_sigterm_handler_ = std::signal(SIGTERM, &rosbag2_py::Player::signal_handler);
    old_sigint_handler_ = std::signal(SIGINT, &rosbag2_py::Player::signal_handler);
  }

  static void uninstall_signal_handlers()
  {
    if (old_sigterm_handler_ != SIG_ERR) {
      std::signal(SIGTERM, old_sigterm_handler_);
      old_sigterm_handler_ = SIG_ERR;
    }
    if (old_sigint_handler_ != SIG_ERR) {
      std::signal(SIGINT, old_sigint_handler_);
      old_sigint_handler_ = SIG_ERR;
    }
    deferred_sig_number_ = -1;
  }

  static void process_deferred_signal()
  {
    auto call_signal_handler = [](const SignalHandlerType & signal_handler, int sig_num) {
        if (signal_handler != SIG_ERR && signal_handler != SIG_IGN && signal_handler != SIG_DFL) {
          signal_handler(sig_num);
        }
      };

    if (deferred_sig_number_ == SIGINT) {
      call_signal_handler(old_sigint_handler_, deferred_sig_number_);
    } else if (deferred_sig_number_ == SIGTERM) {
      call_signal_handler(old_sigterm_handler_, deferred_sig_number_);
    }
  }

  void play_impl(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options,
    bool burst = false,
    size_t burst_num_messages = 0)
  {
    install_signal_handlers();
    try {
      auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      std::shared_ptr<KeyboardHandler> keyboard_handler;
      if (!play_options.disable_keyboard_controls) {
#ifndef _WIN32
        // Instantiate KeyboardHandler explicitly with disabled signal handler inside,
        // since we have already installed our own signal handler
        keyboard_handler = std::make_shared<KeyboardHandler>(false);
#else
        // We don't have signal handler option in constructor for windows version
        keyboard_handler = std::shared_ptr<KeyboardHandler>(new KeyboardHandler());
#endif
      }
      auto player = std::make_shared<rosbag2_transport::Player>(
        std::move(reader), std::move(keyboard_handler), storage_options, play_options);

      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(player);
      auto spin_thread = std::thread(
        [&exec]() {
          exec.spin();
        });
      player->play();

      auto wait_for_exit_thread = std::thread(
        [&]() {
          std::unique_lock<std::mutex> lock(wait_for_exit_mutex_);
          wait_for_exit_cv_.wait(lock, [] {return rosbag2_py::Player::exit_.load();});
          player->stop();
        });
      {
        // Release the GIL for long-running play, so that calling Python code
        // can use other threads
        py::gil_scoped_release release;
        if (burst) {
          player->burst(burst_num_messages);
        }
        player->wait_for_playback_to_finish();
      }

      rosbag2_py::Player::cancel();  // Need to trigger exit from wait_for_exit_thread
      if (wait_for_exit_thread.joinable()) {
        wait_for_exit_thread.join();
      }

      exec.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      exec.remove_node(player);
    } catch (...) {
      process_deferred_signal();
      uninstall_signal_handlers();
      throw;
    }
    process_deferred_signal();
    uninstall_signal_handlers();
  }

  static std::atomic_bool exit_;
  static std::condition_variable wait_for_exit_cv_;
  static SignalHandlerType old_sigint_handler_;
  static SignalHandlerType old_sigterm_handler_;
  static int deferred_sig_number_;
  std::mutex wait_for_exit_mutex_;
};

Player::SignalHandlerType Player::old_sigint_handler_ {SIG_ERR};
Player::SignalHandlerType Player::old_sigterm_handler_ {SIG_ERR};
int Player::deferred_sig_number_{-1};
std::atomic_bool Player::exit_{false};
std::condition_variable Player::wait_for_exit_cv_{};

class Recorder
{
public:
  using SignalHandlerType = void (*)(int);
  Recorder()
  {
    rclcpp::init(0, nullptr);
  }

  virtual ~Recorder()
  {
    rclcpp::shutdown();
  }

  void record(
    const rosbag2_storage::StorageOptions & storage_options,
    RecordOptions & record_options,
    std::string & node_name)
  {
    install_signal_handlers();
    try {
      exit_ = false;
      auto exec = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      if (record_options.rmw_serialization_format.empty()) {
        record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
      }

      auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
      std::shared_ptr<KeyboardHandler> keyboard_handler;
      if (!record_options.disable_keyboard_controls) {
#ifndef _WIN32
        // Instantiate KeyboardHandler explicitly with disabled signal handler inside,
        // since we have already installed our own signal handler
        keyboard_handler = std::make_shared<KeyboardHandler>(false);
#else
        // We don't have signal handler option in constructor for windows version
        keyboard_handler = std::shared_ptr<KeyboardHandler>(new KeyboardHandler());
#endif
      }

      auto recorder = std::make_shared<rosbag2_transport::Recorder>(
        std::move(writer), std::move(keyboard_handler), storage_options, record_options, node_name);
      recorder->record();

      exec->add_node(recorder);
      // Run exec->spin() in a separate thread, because we need to call exec->cancel() after
      // recorder->stop() to be able to send notifications about bag split and close.
      auto spin_thread = std::thread(
        [&exec]() {
          exec->spin();
        });
      {
        // Release the GIL for long-running record, so that calling Python code
        // can use other threads
        py::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(wait_for_exit_mutex_);
        wait_for_exit_cv_.wait(lock, [] {return rosbag2_py::Recorder::exit_.load();});
        recorder->stop();
      }
      exec->cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
      exec->remove_node(recorder);
    } catch (...) {
      process_deferred_signal();
      uninstall_signal_handlers();
      throw;
    }
    process_deferred_signal();
    uninstall_signal_handlers();
  }

  static void cancel()
  {
    exit_ = true;
    wait_for_exit_cv_.notify_all();
  }

protected:
  static void signal_handler(int sig_num)
  {
    if (sig_num == SIGINT || sig_num == SIGTERM) {
      deferred_sig_number_ = sig_num;
      rosbag2_py::Recorder::cancel();
    }
  }

  static void install_signal_handlers()
  {
    deferred_sig_number_ = -1;
    old_sigterm_handler_ = std::signal(SIGTERM, &rosbag2_py::Recorder::signal_handler);
    old_sigint_handler_ = std::signal(SIGINT, &rosbag2_py::Recorder::signal_handler);
  }

  static void uninstall_signal_handlers()
  {
    if (old_sigterm_handler_ != SIG_ERR) {
      std::signal(SIGTERM, old_sigterm_handler_);
      old_sigterm_handler_ = SIG_ERR;
    }
    if (old_sigint_handler_ != SIG_ERR) {
      std::signal(SIGINT, old_sigint_handler_);
      old_sigint_handler_ = SIG_ERR;
    }
    deferred_sig_number_ = -1;
  }

  static void process_deferred_signal()
  {
    auto call_signal_handler = [](const SignalHandlerType & signal_handler, int sig_num) {
        if (signal_handler != SIG_ERR && signal_handler != SIG_IGN && signal_handler != SIG_DFL) {
          signal_handler(sig_num);
        }
      };

    if (deferred_sig_number_ == SIGINT) {
      call_signal_handler(old_sigint_handler_, deferred_sig_number_);
    } else if (deferred_sig_number_ == SIGTERM) {
      call_signal_handler(old_sigterm_handler_, deferred_sig_number_);
    }
  }

  static std::atomic_bool exit_;
  static std::condition_variable wait_for_exit_cv_;
  static SignalHandlerType old_sigint_handler_;
  static SignalHandlerType old_sigterm_handler_;
  static int deferred_sig_number_;
  std::mutex wait_for_exit_mutex_;
};

Recorder::SignalHandlerType Recorder::old_sigint_handler_ {SIG_ERR};
Recorder::SignalHandlerType Recorder::old_sigterm_handler_ {SIG_ERR};
int Recorder::deferred_sig_number_{-1};
std::atomic_bool Recorder::exit_{false};
std::condition_variable Recorder::wait_for_exit_cv_{};

// Return a RecordOptions struct with defaults set for rewriting bags.
rosbag2_transport::RecordOptions bag_rewrite_default_record_options()
{
  rosbag2_transport::RecordOptions options{};
  // We never want to drop messages when converting bags, so set the compression queue size to 0
  // (unbounded).
  options.compression_queue_size = 0;
  return options;
}

// Simple wrapper to read the output config YAML into structs
void bag_rewrite(
  const std::vector<rosbag2_storage::StorageOptions> & input_options,
  std::string output_config_file)
{
  YAML::Node yaml_file = YAML::LoadFile(output_config_file);
  auto bag_nodes = yaml_file["output_bags"];
  if (!bag_nodes) {
    throw std::runtime_error("Output bag config YAML file must have top-level key 'output_bags'");
  }
  if (!bag_nodes.IsSequence()) {
    throw std::runtime_error(
            "Top-level key 'output_bags' must contain a list of "
            "StorageOptions/RecordOptions dicts.");
  }

  std::vector<
    std::pair<rosbag2_storage::StorageOptions, rosbag2_transport::RecordOptions>> output_options;
  for (const auto & bag_node : bag_nodes) {
    rosbag2_storage::StorageOptions storage_options{};
    YAML::convert<rosbag2_storage::StorageOptions>::decode(bag_node, storage_options);
    rosbag2_transport::RecordOptions record_options = bag_rewrite_default_record_options();
    record_options = bag_node.as<rosbag2_transport::RecordOptions>();
    output_options.push_back(std::make_pair(storage_options, record_options));
  }
  rosbag2_transport::bag_rewrite(input_options, output_options);
}

}  // namespace rosbag2_py

PYBIND11_MODULE(_transport, m) {
  m.doc() = "Python wrapper of the rosbag2_transport API";

  // NOTE: it is non-trivial to add a constructor for PlayOptions and RecordOptions
  // because the rclcpp::QoS <-> rclpy.qos.QoS Profile conversion cannot be done by builtins.
  // It is possible, but the code is much longer and harder to maintain, requiring duplicating
  // the names of the members multiple times, as well as the default values from the struct
  // definitions.

  py::class_<PlayOptions>(m, "PlayOptions")
  .def(py::init<>())
  .def_readwrite("read_ahead_queue_size", &PlayOptions::read_ahead_queue_size)
  .def_readwrite("node_prefix", &PlayOptions::node_prefix)
  .def_readwrite("rate", &PlayOptions::rate)
  .def_readwrite("topics_to_filter", &PlayOptions::topics_to_filter)
  .def_readwrite("services_to_filter", &PlayOptions::services_to_filter)
  .def_readwrite("regex_to_filter", &PlayOptions::regex_to_filter)
  .def_readwrite("exclude_regex_to_filter", &PlayOptions::exclude_regex_to_filter)
  .def_readwrite("exclude_topics_to_filter", &PlayOptions::exclude_topics_to_filter)
  .def_readwrite("exclude_service_events_to_filter", &PlayOptions::exclude_services_to_filter)
  .def_property(
    "topic_qos_profile_overrides",
    &PlayOptions::getTopicQoSProfileOverrides,
    &PlayOptions::setTopicQoSProfileOverrides)
  .def_readwrite("loop", &PlayOptions::loop)
  .def_readwrite("topic_remapping_options", &PlayOptions::topic_remapping_options)
  .def_readwrite("clock_publish_frequency", &PlayOptions::clock_publish_frequency)
  .def_readwrite("clock_publish_on_topic_publish", &PlayOptions::clock_publish_on_topic_publish)
  .def_readwrite("clock_topics", &PlayOptions::clock_trigger_topics)
  .def_property(
    "delay",
    &PlayOptions::getDelay,
    &PlayOptions::setDelay)
  .def_property(
    "playback_duration",
    &PlayOptions::getPlaybackDuration,
    &PlayOptions::setPlaybackDuration)
  .def_readwrite("disable_keyboard_controls", &PlayOptions::disable_keyboard_controls)
  .def_readwrite("start_paused", &PlayOptions::start_paused)
  .def_property(
    "start_offset",
    &PlayOptions::getStartOffset,
    &PlayOptions::setStartOffset)
  .def_property(
    "playback_until_timestamp",
    &PlayOptions::getPlaybackUntilTimestamp,
    &PlayOptions::setPlaybackUntilTimestamp)
  .def_readwrite("wait_acked_timeout", &PlayOptions::wait_acked_timeout)
  .def_readwrite("disable_loan_message", &PlayOptions::disable_loan_message)
  .def_readwrite("publish_service_requests", &PlayOptions::publish_service_requests)
  .def_readwrite("service_requests_source", &PlayOptions::service_requests_source)
  ;

  py::enum_<rosbag2_transport::ServiceRequestsSource>(m, "ServiceRequestsSource")
  .value("SERVICE_INTROSPECTION", rosbag2_transport::ServiceRequestsSource::SERVICE_INTROSPECTION)
  .value("CLIENT_INTROSPECTION", rosbag2_transport::ServiceRequestsSource::CLIENT_INTROSPECTION)
  ;

  py::class_<RecordOptions>(m, "RecordOptions")
  .def(py::init<>())
  .def_readwrite("all_topics", &RecordOptions::all_topics)
  .def_readwrite("is_discovery_disabled", &RecordOptions::is_discovery_disabled)
  .def_readwrite("topics", &RecordOptions::topics)
  .def_readwrite("topic_types", &RecordOptions::topic_types)
  .def_readwrite("exclude_topic_types", &RecordOptions::exclude_topic_types)
  .def_readwrite("rmw_serialization_format", &RecordOptions::rmw_serialization_format)
  .def_readwrite("topic_polling_interval", &RecordOptions::topic_polling_interval)
  .def_readwrite("regex", &RecordOptions::regex)
  .def_readwrite("exclude_regex", &RecordOptions::exclude_regex)
  .def_readwrite("exclude_topics", &RecordOptions::exclude_topics)
  .def_readwrite("exclude_service_events", &RecordOptions::exclude_service_events)
  .def_readwrite("node_prefix", &RecordOptions::node_prefix)
  .def_readwrite("compression_mode", &RecordOptions::compression_mode)
  .def_readwrite("compression_format", &RecordOptions::compression_format)
  .def_readwrite("compression_queue_size", &RecordOptions::compression_queue_size)
  .def_readwrite("compression_threads", &RecordOptions::compression_threads)
  .def_property(
    "topic_qos_profile_overrides",
    &RecordOptions::getTopicQoSProfileOverrides,
    &RecordOptions::setTopicQoSProfileOverrides)
  .def_readwrite("include_hidden_topics", &RecordOptions::include_hidden_topics)
  .def_readwrite("include_unpublished_topics", &RecordOptions::include_unpublished_topics)
  .def_readwrite("start_paused", &RecordOptions::start_paused)
  .def_readwrite("ignore_leaf_topics", &RecordOptions::ignore_leaf_topics)
  .def_readwrite("use_sim_time", &RecordOptions::use_sim_time)
  .def_readwrite("services", &RecordOptions::services)
  .def_readwrite("all_services", &RecordOptions::all_services)
  .def_readwrite("disable_keyboard_controls", &RecordOptions::disable_keyboard_controls)
  ;

  py::class_<rosbag2_py::Player>(m, "Player")
  .def(py::init())
  .def("play", &rosbag2_py::Player::play, py::arg("storage_options"), py::arg("play_options"))
  .def(
    "burst", &rosbag2_py::Player::burst, py::arg("storage_options"), py::arg("play_options"),
    py::arg("num_messages"))
  .def_static("cancel", &rosbag2_py::Player::cancel)
  ;

  py::class_<rosbag2_py::Recorder>(m, "Recorder")
  .def(py::init())
  .def(
    "record", &rosbag2_py::Recorder::record, py::arg("storage_options"), py::arg("record_options"),
    py::arg("node_name") = "rosbag2_recorder")
  .def_static("cancel", &rosbag2_py::Recorder::cancel)
  ;

  m.def(
    "bag_rewrite",
    &rosbag2_py::bag_rewrite,
    "Given one or more input bags, output one or more bags with new settings.");
}

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

#include <algorithm>
#include <csignal>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "keyboard_handler/keyboard_handler.hpp"
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

class Arguments
{
public:
  explicit Arguments(const std::vector<std::string> & args)
  : arguments_(args)
  {
    std::for_each(
      arguments_.begin(), arguments_.end(),
      [this](const std::string & arg) {
        pointers_.push_back(arg.c_str());
      }
    );
    pointers_.push_back(nullptr);
  }

  const char ** argv()
  {
    return arguments_.empty() ? nullptr : pointers_.data();
  }

  [[nodiscard]] int argc() const
  {
    return static_cast<int>(arguments_.size());
  }

private:
  std::vector<std::string> arguments_;
  std::vector<const char *> pointers_;
};

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

  // TODO(christophebedard): remove this constructor after a deprecation period
  // along with the `if (!player_)` checks in the methods
  explicit Player(const std::string & log_level = "info")
  {
    Arguments arguments({"--ros-args", "--log-level", log_level});
    rclcpp::init(arguments.argc(), arguments.argv());
    // Intentionally not initializing player_ here, because default constructor is intended to be
    // used for the composable node only since it will call Player::Play() inside. Also it will
    // fail without specifying storage options with valid path to the bag file via ros args.
  }

  Player(
    const rosbag2_storage::StorageOptions & storage_options,
    const PlayOptions & play_options,
    const std::string & log_level,
    const std::string & node_name)
  : Player(
      std::vector<rosbag2_storage::StorageOptions>({storage_options}), play_options, log_level,
      node_name) {}

  Player(
    const std::vector<rosbag2_storage::StorageOptions> & storage_options,
    const PlayOptions & play_options,
    const std::string & log_level,
    const std::string & node_name)
  {
    Arguments arguments({"--ros-args", "--log-level", log_level});
    // Don't install signal handlers to keep signal handling simple in the Python layer
    rclcpp::init(arguments.argc(), arguments.argv(),
                 rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    player_ = std::make_shared<rosbag2_transport::Player>(storage_options, play_options, node_name);
  }

  virtual ~Player()
  {
    stop_spin();
    rclcpp::shutdown();
  }

  void start_spin()
  {
    std::lock_guard<std::mutex> lock(spin_thread_mutex_);
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    if (exec_) {
      // We already have an executor spinning
      return;
    }
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(player_);
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });
    // Wait for the executor to start spinning to avoid race conditions
    while (!exec_->is_spinning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void stop_spin()
  {
    std::lock_guard<std::mutex> lock(spin_thread_mutex_);
    if (exec_) {
      exec_->cancel();
      if (spin_thread_.joinable()) {
        spin_thread_.join();
      }
      exec_->remove_node(player_);
      exec_ = nullptr;
    }
  }

  void play()
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    player_->play();
  }

  bool wait_for_playback_to_start(double timeout = -1.0)
  {
    return wait_for_async(
      timeout,
      [&](double timeout) {
        return player_->wait_for_playback_to_start(std::chrono::duration<double>(timeout));
      });
  }

  bool wait_for_playback_to_start_exclusively(double timeout = -1.0)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
                               "storage and play options.");
    }
    return player_->wait_for_playback_to_start(std::chrono::duration<double>(timeout));
  }

  bool wait_for_playback_to_finish(double timeout = -1.0)
  {
    return wait_for_async(
      timeout,
      [&](double timeout) {
        return player_->wait_for_playback_to_finish(std::chrono::duration<double>(timeout));
      });
  }

  bool wait_for_playback_to_finish_exclusively(double timeout = -1.0)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
                               "storage and play options.");
    }
    return player_->wait_for_playback_to_finish(std::chrono::duration<double>(timeout));
  }

  void stop()
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    player_->stop();
  }

  void pause()
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    player_->pause();
  }

  void resume()
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    player_->resume();
  }

  [[nodiscard]] bool is_paused() const
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    return player_->is_paused();
  }

  [[nodiscard]] int64_t get_starting_time() const
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    return player_->get_starting_time();
  }

  [[nodiscard]] int64_t get_playback_duration() const
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    return player_->get_playback_duration();
  }

  bool play_next()
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    return player_->play_next();
  }

  size_t burst(size_t num_messages)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    return player_->burst(num_messages);
  }

  void seek(int64_t time_point)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    player_->seek(static_cast<rcutils_time_point_value_t>(time_point));
  }

  static void cancel()
  {
    PyErr_WarnEx(PyExc_DeprecationWarning,
                 "Player.cancel() is deprecated. Please use Player.stop() instead.", 1);
    exit_ = true;
    wait_for_exit_cv_.notify_all();
  }

  // TODO(christophebedard): remove this method after a deprecation period
  void play(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options)
  {
    player_ = std::make_shared<rosbag2_transport::Player>(storage_options, play_options);
    play_impl(false);
  }

  // TODO(christophebedard): remove this method after a deprecation period
  void play(
    const std::vector<rosbag2_storage::StorageOptions> & storage_options,
    PlayOptions & play_options)
  {
    player_ = std::make_shared<rosbag2_transport::Player>(storage_options, play_options);
    play_impl(false);
  }

  // TODO(christophebedard): remove this method after a deprecation period
  void burst(
    const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options,
    size_t num_messages)
  {
    player_ = std::make_shared<rosbag2_transport::Player>(storage_options, play_options);
    play_impl(true, num_messages);
  }

protected:
  bool wait_for_async(double timeout, const std::function<bool(double)> & wait_for_function)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }

    const double wait_period =
      (timeout < 0.0) ? SIGNAL_CHECK_INTERVAL : std::min(timeout, SIGNAL_CHECK_INTERVAL);

    // We need to release the Global Interpreter Lock (GIL) to allow other Python threads to
    // proceed. However, the GIL release doesn't result in Python checking for signals while
    // we're in C++. We will wait for a short period of time and then check for signals in the loop.
    // If any signals (Ctrl+C/SIGINT or SIGTERM) have been received, we will propagate them to
    // Python.
    bool finished = false;
    while (!finished) {
      {
        // Release the Global Interpreter Lock (GIL) to allow other Python threads to proceed
        py::gil_scoped_release release;
        finished = wait_for_function(wait_period);
      }
      // The GIL release doesn't result in Python checking for signals while we're in C++, so we can
      // do it here to simplify the Python layer: just periodically check for signals while waiting
      // Wait for a short period of time
      {
        py::gil_scoped_acquire gil;
        // If a signal (Ctrl+C/SIGINT or SIGTERM) has been received, propagate it to Python
        if (PyErr_CheckSignals() != 0) {
          throw py::error_already_set();
        }
      }
      // If we are not done yet and have a non-infinite timeout value
      if (!finished && timeout > 0.0) {
        // Decrement the timeout that's left
        timeout -= wait_period;
        // If we waited all the way to the timeout (and aren't done), return false
        if (timeout <= 0.0) {
          return false;
        }
      }
    }
    return finished;
  }

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

  void play_impl(bool burst = false, size_t burst_num_messages = 0)
  {
    if (!player_) {
      throw std::runtime_error("Player is not initialized. Please use constructor with "
        "storage and play options.");
    }
    install_signal_handlers();
    try {
      start_spin();
      player_->play();

      auto wait_for_exit_thread = std::thread(
        [this]() {
          std::unique_lock<std::mutex> lock(wait_for_exit_mutex_);
          wait_for_exit_cv_.wait(lock, [] {return rosbag2_py::Player::exit_.load();});
          player_->stop();
        });
      {
        // Release the GIL for long-running play, so that calling Python code
        // can use other threads
        py::gil_scoped_release release;
        if (burst) {
          player_->burst(burst_num_messages);
        }
        player_->wait_for_playback_to_finish();
      }

      rosbag2_py::Player::cancel();  // Need to trigger exit from wait_for_exit_thread
      if (wait_for_exit_thread.joinable()) {
        wait_for_exit_thread.join();
      }

      stop_spin();
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
  static constexpr double SIGNAL_CHECK_INTERVAL = 0.1;
  std::mutex wait_for_exit_mutex_;

  std::shared_ptr<rosbag2_transport::Player> player_;
  std::mutex spin_thread_mutex_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_{nullptr};
  std::thread spin_thread_;
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

  // TODO(christophebedard): remove this constructor after a deprecation period
  // along with the `if (!recorder_)` checks in the methods
  explicit Recorder(const std::string & log_level = "info")
  {
    Arguments arguments({"--ros-args", "--log-level", log_level});
    rclcpp::init(arguments.argc(), arguments.argv());
    // Intentionally not initializing recorder_ here, because default constructor is intended to be
    // used for the composable node only since it will call Recorder::record() inside.
  }

  Recorder(
    const rosbag2_storage::StorageOptions & storage_options,
    RecordOptions & record_options,
    const std::string & log_level,
    const std::string & node_name)
  {
    Arguments arguments({"--ros-args", "--log-level", log_level});
    // Don't install signal handlers to keep signal handling simple in the Python layer
    rclcpp::init(arguments.argc(), arguments.argv(),
                 rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

    if (record_options.rmw_serialization_format.empty()) {
      record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
    }
    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);

    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer), storage_options, record_options, node_name);
  }

  virtual ~Recorder()
  {
    stop_spin();
    rclcpp::shutdown();
  }

  void start_spin()
  {
    std::lock_guard<std::mutex> lock(spin_thread_mutex_);
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    if (exec_) {
      // We already have an executor spinning
      return;
    }
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(recorder_);
    spin_thread_ = std::thread(
      [this]() {
        exec_->spin();
      });
    // Wait for the executor to start spinning to avoid race conditions
    while (!exec_->is_spinning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void stop_spin()
  {
    std::lock_guard<std::mutex> lock(spin_thread_mutex_);
    if (exec_) {
      exec_->cancel();
      if (spin_thread_.joinable()) {
        spin_thread_.join();
      }
      exec_->remove_node(recorder_);
      exec_ = nullptr;
    }
  }

  void record()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    recorder_->record();
  }

  void stop()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    recorder_->stop();
  }

  void pause()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    recorder_->pause();
  }

  void resume()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    recorder_->resume();
  }

  bool is_paused()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    return recorder_->is_paused();
  }

  // TODO(christophebedard): remove this method after a deprecation period
  void record(
    const rosbag2_storage::StorageOptions & storage_options,
    RecordOptions & record_options,
    const std::string & node_name)
  {
    if (record_options.rmw_serialization_format.empty()) {
      record_options.rmw_serialization_format = std::string(rmw_get_serialization_format());
    }
    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);

    recorder_ = std::make_shared<rosbag2_transport::Recorder>(
      std::move(writer), storage_options, record_options, node_name);

    record_impl();
  }

  void record_impl()
  {
    if (!recorder_) {
      throw std::runtime_error("Recorder is not initialized. Please use constructor with "
        "storage and record options.");
    }
    install_signal_handlers();
    try {
      exit_ = false;
      recorder_->record();
      // Run exec->spin() in a separate thread, because we need to call exec->cancel() after
      // recorder->stop() to be able to send notifications about bag split and close.
      start_spin();
      {
        // Release the GIL for long-running record, so that calling Python code
        // can use other threads
        py::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(wait_for_exit_mutex_);
        wait_for_exit_cv_.wait(lock, [] {return rosbag2_py::Recorder::exit_.load();});
        recorder_->stop();
      }

      stop_spin();
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
    PyErr_WarnEx(PyExc_DeprecationWarning,
                 "Recorder.cancel() is deprecated. Please use Recorder.stop() instead.", 1);
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

  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  std::mutex spin_thread_mutex_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_{nullptr};
  std::thread spin_thread_;
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
    YAML::convert<rosbag2_transport::RecordOptions>::decode(bag_node, record_options);
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
  .def_readwrite("compression_threads_priority", &RecordOptions::compression_threads_priority)
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
  // TODO(christophebedard): remove this constructor after a deprecation period
  // Deprecated default constructor
  .def(py::init([]()
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Player() is deprecated. Use the constructor with "
          "full configuration parameters instead.", 1);
      return new rosbag2_py::Player();
    }), "Deprecated: Use constructor with full options.")

  // TODO(christophebedard): remove this constructor after a deprecation period
  // Deprecated constructor with log_level
  .def(py::init([](const std::string & log_level)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Player(log_level) is deprecated. Use the "
          "constructor with full configuration parameters instead.", 1);
      return new rosbag2_py::Player(log_level);
    }), py::arg("log_level"), "Deprecated: Use constructor with full options.")

  // Recommended constructor with storage and play options
  .def(py::init<const rosbag2_storage::StorageOptions &, const PlayOptions &,
    const std::string &, const std::string &>(),
    py::arg("storage_options"),
    py::arg("play_options"),
    py::arg("log_level") = "info",
    py::arg("node_name") = "rosbag2_player",
    R"pbdoc(
      Initialize a Player with complete configuration.

      Args:
          storage_options (StorageOptions): Configuration for storage backend (e.g., URI, format).
          play_options (PlayOptions): Options for playback (e.g., topics, QoS settings).
          log_level (str, optional): Logging level, defaults to 'info'.
          node_name (str, optional): Name of the player node, defaults to 'rosbag2_player'.
    )pbdoc")

  // Recommended constructor with multiple storage options
  .def(py::init<const std::vector<rosbag2_storage::StorageOptions> &, const PlayOptions &,
    const std::string &, const std::string &>(),
    py::arg("storage_options"),
    py::arg("play_options"),
    py::arg("log_level") = "info",
    py::arg("node_name") = "rosbag2_player",
    R"pbdoc(
      Initialize a Player with multiple storage options.

      Args:
          storage_options (List[StorageOptions]): List of storage configurations.
          play_options (PlayOptions): Options for playback (e.g., topics, QoS settings).
          log_level (str, optional): Logging level, defaults to 'info'.
          node_name (str, optional): Name of the player node, defaults to 'rosbag2_player'.
    )pbdoc")

  .def("start_spin", &rosbag2_py::Player::start_spin,
    R"pbdoc(
      Create and start a thread with an executor that spins the player node.

      Waits for the executor to start spinning before returning.
    )pbdoc")

  .def("stop_spin", &rosbag2_py::Player::stop_spin,
    R"pbdoc(
      Stop the thread that spins the player node, if there is one.

      This is automatically called by the destructor, if needed.
    )pbdoc")

  // Recommended play method
  .def("play", py::overload_cast<>(&rosbag2_py::Player::play),
    R"pbdoc(
      Start playback based on the internal Player configuration.

      This is the preferred method for starting playback.
      It is equivalent to the rosbag2_transport::Player::player() method.
      All parameters should be configured via the constructor.
    )pbdoc")

  .def("wait_for_playback_to_start",
    &rosbag2_py::Player::wait_for_playback_to_start,
    py::arg("timeout") = -1.0,
    R"pbdoc(
      Releasing Global Interpreter Lock (GIL) to allow other Python threads to proceed then wait
      for the playback to start and for the message queue to be filled. Also periodically checks for
      signals (Ctrl+C/SIGINT or SIGTERM) and propagates them to Python.

      Args:
          timeout (float): Maximum time to wait in seconds. Default is -1 (wait indefinitely).
      Returns:
          bool: True if playback started successfully, False if timed out.
    )pbdoc")

  .def("wait_for_playback_to_start_exclusively",
    &rosbag2_py::Player::wait_for_playback_to_start_exclusively,
    py::arg("timeout") = -1.0,
    R"pbdoc(
      Wait for the playback to start exclusively without releasing Global Interpreter Lock (GIL).

      Args:
          timeout (float): Maximum time to wait in seconds. Default is -1 (wait indefinitely).
      Returns:
          bool: True if playback started successfully, False if timed out.
    )pbdoc")

  .def("wait_for_playback_to_finish",
    &rosbag2_py::Player::wait_for_playback_to_finish,
    py::arg("timeout") = -1.0,
    R"pbdoc(
      Releasing Global Interpreter Lock (GIL) to allow other Python threads to proceed then wait
      for the playback to finish.  Also periodically checks for signals (Ctrl+C/SIGINT or SIGTERM)
      and propagates them to Python.

      Args:
          timeout (float): Maximum time to wait in seconds. Default is -1 (wait indefinitely).
      Returns:
          bool: True if playback finished successfully, False if timed out.
    )pbdoc")

  .def("wait_for_playback_to_finish_exclusively",
    &rosbag2_py::Player::wait_for_playback_to_finish_exclusively,
    py::arg("timeout") = -1.0,
    R"pbdoc(
      Wait for the playback to finish exclusively without releasing Global Interpreter Lock (GIL).

      Args:
          timeout (float): Maximum time to wait in seconds. Default is -1 (wait indefinitely).
      Returns:
          bool: True if playback finished successfully, False if timed out.
    )pbdoc")

  .def(
    "stop",
    &rosbag2_py::Player::stop,
    "Unpause if in pause mode, stop playback and exit from play.")

  .def(
    "pause",
    &rosbag2_py::Player::pause,
    "Pause the flow of time for playback.")

  .def(
    "resume",
    &rosbag2_py::Player::resume,
    "Start the flow of time for playback.")

  .def(
    "is_paused",
    &rosbag2_py::Player::is_paused,
    "Whether the playback is currently paused.")

  .def("get_starting_time",
    &rosbag2_py::Player::get_starting_time,
    R"pbdoc(
      Get the starting time of the playback.

      Returns:
          int: The timestamp of the first message in nanoseconds.
    )pbdoc")

  .def("get_playback_duration",
    &rosbag2_py::Player::get_playback_duration,
    R"pbdoc(
      Get the total duration of the playback.

      Returns:
          int: The total duration of the playback in nanoseconds.
    )pbdoc")

  .def("play_next", &rosbag2_py::Player::play_next,
    R"pbdoc(
      Playing next message from queue when in pause.

      Returns:
          bool: True if a message was played, False if no more messages are available.
    )pbdoc")

  .def("seek", &rosbag2_py::Player::seek, py::arg("time_point"),
    R"pbdoc(
      Advance player to the message with closest timestamp >= time_point.

      Args:
          time_point (int): Time point in ROS playback timeline, in nanoseconds.
    )pbdoc")

  // TODO(christophebedard): remove this method after a deprecation period
  // Deprecated play method with storage and play options
  .def("play",
    [](rosbag2_py::Player & self, const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Player.play(storage_options, play_options) is "
          "deprecated. Use the parameterless play() instead.", 1);
      return self.play(storage_options, play_options);
    },
    py::arg("storage_options"),
    py::arg("play_options"),
    "Deprecated: use play() with preconfigured options instead.")

  // TODO(christophebedard): remove this method after a deprecation period
  // Deprecated play method with multiple storage options
  .def("play",
    [](rosbag2_py::Player & self,
    const std::vector<rosbag2_storage::StorageOptions> & storage_options,
    PlayOptions & play_options)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Player.play(storage_options_list, play_options) is "
          "deprecated. Use the parameterless play() instead.", 1);
      return self.play(storage_options, play_options);
    },
    py::arg("storage_options"),
    py::arg("play_options"),
    "Deprecated: use play() with preconfigured options instead.")

  // Recommended burst playback method
  .def("burst", py::overload_cast<size_t>(&rosbag2_py::Player::burst), py::arg("num_messages"),
    R"pbdoc(
      Play a burst of messages.

      This is equivalent to the rosbag2_transport::Player::burst(const size_t) method.

      Args:
          num_messages (int): Number of messages to play in this burst.
      Returns:
          size_t: Number of messages played in this burst.
    )pbdoc")

  // TODO(christophebedard): remove this method after a deprecation period
  // Deprecated burst method with storage and play options
  .def("burst",
    [](rosbag2_py::Player & self, const rosbag2_storage::StorageOptions & storage_options,
    PlayOptions & play_options, size_t num_messages)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning,
          "Player.burst(storage_options, play_options, num_messages) is deprecated. "
          "Use burst(num_messages) with preconfigured options instead.", 1);
      return self.burst(storage_options, play_options, num_messages);
    },
    py::arg("storage_options"),
    py::arg("play_options"),
    py::arg("num_messages"),
    "Deprecated: use burst(num_messages) with preconfigured options instead.")

  .def_static("cancel", &rosbag2_py::Player::cancel,
    R"pbdoc(
      Cancel the ongoing playback session.

      This is a static method and will affect any running Players globally.
      Deprecated: use Player.stop() instead.
    )pbdoc")
  ;

  py::class_<rosbag2_py::Recorder>(m, "Recorder")
  // TODO(christophebedard): remove this constructor after a deprecation period
  // Deprecated default constructor
  .def(py::init([]()
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Recorder() is deprecated. Use the constructor with "
        "full configuration parameters instead.", 1);
      return new rosbag2_py::Recorder();
    }), "Deprecated: Use constructor with full options.")

  // TODO(christophebedard): remove this constructor after a deprecation period
  // Deprecated constructor with string argument
  .def(py::init([](const std::string & arg)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Recorder(log_level) is deprecated. Use the"
        " constructor with full configuration parameters instead.", 1);
      return new rosbag2_py::Recorder(arg);
    }), py::arg("arg"), "Deprecated: Use constructor with full options.")

  // Recommended constructor with storage and record options
  .def(
    py::init<const rosbag2_storage::StorageOptions &, RecordOptions &,
    const std::string &, const std::string &>(),
    py::arg("storage_options"),
    py::arg("record_options"),
    py::arg("log_level") = "info",
    py::arg("node_name") = "rosbag2_recorder",
    R"pbdoc(
      Initialize a Recorder with complete configuration.

      Args:
          storage_options (StorageOptions): Configuration for storage backend (e.g., URI, format).
          record_options (RecordOptions): Options for recording (e.g., topics, QoS settings).
          log_level (str, optional): Logging level, defaults to 'info'.
          node_name (str, optional): Name of the recorder node, defaults to 'rosbag2_recorder'.
    )pbdoc")

  .def("start_spin", &rosbag2_py::Recorder::start_spin,
    R"pbdoc(
      Create and start a thread with an executor that spins the recorder node.

      Waits for the executor to start spinning before returning.
    )pbdoc")

  .def("stop_spin", &rosbag2_py::Recorder::stop_spin,
    R"pbdoc(
      Stop the thread that spins the recorder node, if there is one.
    )pbdoc")

  .def("record", py::overload_cast<>(&rosbag2_py::Recorder::record),
    R"pbdoc(
      Start recording based on the internal Recorder configuration.

      This is the preferred method for starting a recording session.
      It is equivalent to the rosbag2_transport::Recorder::record() method.
      All parameters should be configured via the constructor.
    )pbdoc")

  .def(
    "stop",
    &rosbag2_py::Recorder::stop,
    "Stop recording.")

  .def(
    "pause",
    &rosbag2_py::Recorder::pause,
    "Pause the recording.")

  .def(
    "resume",
    &rosbag2_py::Recorder::resume,
    "Resume the recording.")

  .def(
    "is_paused",
    &rosbag2_py::Recorder::is_paused,
    "Whether the recording is currently paused.")

  // TODO(christophebedard): remove this method after a deprecation period
  // (deprecated) record method
  .def("record",
    [](rosbag2_py::Recorder & self, const rosbag2_storage::StorageOptions & storage_options,
    RecordOptions & record_options, const std::string & node_name)
    {
      PyErr_WarnEx(PyExc_DeprecationWarning, "Recorder.record(storage_options, record_options, "
        "node_name) is deprecated. Use the parameterless record() instead.", 1);
      return self.record(storage_options, record_options, node_name);
    },
    py::arg("storage_options"),
    py::arg("record_options"),
    py::arg("node_name") = "rosbag2_recorder",
    "Deprecated: use record() with preconfigured options instead.")

  .def_static("cancel", &rosbag2_py::Recorder::cancel,
    R"pbdoc(
      Cancel the ongoing recording session.

      This is a static method and will affect any running Recorders globally.
      Deprecated: use Recorder.stop() instead.
    )pbdoc")
  ;

  m.def(
    "bag_rewrite",
    &rosbag2_py::bag_rewrite,
    "Given one or more input bags, output one or more bags with new settings.");
}

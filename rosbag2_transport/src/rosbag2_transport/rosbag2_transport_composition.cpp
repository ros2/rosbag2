// Copyright 2019, Denso ADAS Engineering Services GmbH.
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

#include "rosbag2_transport/rosbag2_transport.hpp"

#include <sys/types.h>  // required for stat.h
#include <sys/stat.h>
#include <memory>
#include <string>

#include "rosbag2_node.hpp"
#include "rosbag2/writer.hpp"
#include "recorder.hpp"

#include "rosbag2_transport/storage_options.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "rosbag2_transport/logging.hpp"
#include "rosbag2_srvs/srv/command_record.hpp"

namespace rosbag2_composition
{

class Rosbag2Node_Recorder : public rosbag2_transport::Rosbag2Node
{
public:
  Rosbag2Node_Recorder()
  : rosbag2_transport::Rosbag2Node("rosbag2_recorder")
  {
    // control recording by a service
    auto handle_command_record =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<rosbag2_srvs::srv::CommandRecord::Request> request,
        std::shared_ptr<rosbag2_srvs::srv::CommandRecord::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(),
          "Setup with %s (%s)", request->uri.c_str(), request->storage_id.c_str());

        // stop recording
        response->success = stop();

        if (!request->uri.empty()) {
          response->success = setup(request->uri, request->storage_id);
        }
      };

    srv_ =
      create_service<rosbag2_srvs::srv::CommandRecord>("command_record", handle_command_record);
  }

  virtual ~Rosbag2Node_Recorder()
  {
    stop();
  }

protected:
  rclcpp::Service<rosbag2_srvs::srv::CommandRecord>::SharedPtr srv_;
  std::shared_ptr<std::thread> thread_;

  /// Stop recording: wait until threads are done and close writer.
  bool stop()
  {
    bool ret = false;

    if (recorder_) {
      ret = recorder_ ? recorder_->stop() : false;
      thread_->join();
      recorder_.reset();
      writer_.reset();
    }

    return ret;
  }

  /// Start recording at the file location (uri) and with the defined storage_id
  bool setup(const std::string & uri, const std::string & storage_id = "sqlite3")
  {
    try {
      rosbag2_transport::StorageOptions storage_options{};
      rosbag2_transport::RecordOptions record_options{};

      storage_options.uri = uri;
      storage_options.storage_id = storage_id;

      // @todo: make configurable
      record_options.all = true;
      record_options.is_discovery_disabled = false;
      record_options.topic_polling_interval = std::chrono::milliseconds(250);
      record_options.rmw_serialization_format = rmw_get_serialization_format();

      // using system functions to reduce build dependencies
      mode_t nMode = 0733;  // UNIX style permissions
      int nError = 0;
#if defined(_WIN32)
      nError = _mkdir(storage_options.uri.c_str());  // can be used on Windows
#else
      nError = mkdir(storage_options.uri.c_str(), nMode);  // can be used on non-Windows
#endif
      if (nError != 0) {
        ROSBAG2_TRANSPORT_LOG_ERROR("Failed to create directory: %s", storage_options.uri.c_str());
      }

      writer_ = std::make_shared<rosbag2::Writer>();
      writer_->open(
        storage_options,
        {rmw_get_serialization_format(), record_options.rmw_serialization_format}
      );

      thread_ = std::make_shared<std::thread>([this, record_options]() {
            recorder_ = std::make_unique<rosbag2_transport::Recorder>(writer_, setup_node());
            recorder_->record(record_options);
          });

      return true;
    } catch (std::runtime_error & e) {
      ROSBAG2_TRANSPORT_LOG_ERROR("Failed to record: %s", e.what());
      return false;
    }

    return true;
  }

  std::shared_ptr<Rosbag2Node> setup_node()
  {
    if (!transport_node_) {
      transport_node_ = std::make_shared<Rosbag2Node>("rosbag2");
    }
    return transport_node_;
  }

  std::shared_ptr<rosbag2::Writer> writer_;
  std::shared_ptr<Rosbag2Node> transport_node_;
  std::unique_ptr<rosbag2_transport::Recorder> recorder_;
};

}  // namespace rosbag2_composition

#include "class_loader/register_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(rosbag2_composition::Rosbag2Node_Recorder, rclcpp::Node)

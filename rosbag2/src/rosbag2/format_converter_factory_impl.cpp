// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include "rosbag2/format_converter_factory_impl.hpp"

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rosbag2/logging.hpp"

namespace rosbag2
{

FormatConverterFactoryImpl::FormatConverterFactoryImpl()
{
  try {
    class_loader_ = std::make_unique<pluginlib::ClassLoader<FormatConverterInterface>>(
      "rosbag2", "rosbag2::FormatConverterInterface");
  } catch (const std::exception & e) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
    throw e;
  }
}

// needed explicit destructor because of unique_ptr for pimpl
FormatConverterFactoryImpl::~FormatConverterFactoryImpl() = default;

std::shared_ptr<FormatConverterInterface> FormatConverterFactoryImpl::load_converter(
  const std::string & format)
{
  auto converter_id = format + "_converter";

  const auto & registered_classes = class_loader_->getDeclaredClasses();
  auto class_exists = std::find(registered_classes.begin(), registered_classes.end(), converter_id);
  if (class_exists == registered_classes.end()) {
    ROSBAG2_LOG_ERROR_STREAM("Requested converter id '" << converter_id << "' does not exist");
    return nullptr;
  }

  try {
    return std::shared_ptr<FormatConverterInterface>(
      class_loader_->createUnmanagedInstance(converter_id));
  } catch (const std::runtime_error & ex) {
    ROSBAG2_LOG_ERROR_STREAM("Unable to load instance of converter interface: " << ex.what());
    return nullptr;
  }
}

}  // namespace rosbag2

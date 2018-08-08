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

#ifndef IMPL__STORAGE_FACTORY_IMPL_HPP_
#define IMPL__STORAGE_FACTORY_IMPL_HPP_

#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rcutils/logging_macros.h"

#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"

namespace rosbag2_storage
{

class StorageFactoryImpl
{
public:
  ~StorageFactoryImpl() = default;

  std::unique_ptr<WritableStorage> get_for_writing(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      writable_class_loader_ = std::make_unique<pluginlib::ClassLoader<WritableStorage>>(
        "rosbag2_storage", "rosbag2_storage::WritableStorage");

      auto registered_classes = writable_class_loader_->getDeclaredClasses();

      for (const auto & class_ : registered_classes) {
        if (class_ == storage_id) {
          auto instance = writable_class_loader_->createUnmanagedInstance(storage_id);
          instance->open_for_writing(uri);
          return std::unique_ptr<WritableStorage>(instance);
        }
      }

      RCUTILS_LOG_WARN("Requested storage id %s does not exist", storage_id.c_str());
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR("Error reading from pluginlib: %s", e.what());
    }

    return std::unique_ptr<WritableStorage>();
  }

  std::unique_ptr<ReadableStorage> get_for_reading(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      readable_class_loader_ = std::make_unique<pluginlib::ClassLoader<ReadableStorage>>(
        "rosbag2_storage", "rosbag2_storage::ReadableStorage");

      auto registered_classes = readable_class_loader_->getDeclaredClasses();

      for (const auto & class_ : registered_classes) {
        if (class_ == storage_id) {
          auto instance = readable_class_loader_->createUnmanagedInstance(storage_id);
          instance->open_for_reading(uri);
          return std::unique_ptr<ReadableStorage>(instance);
        }
      }

      RCUTILS_LOG_WARN("Requested storage id %s does not exist", storage_id.c_str());
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR("Error reading from pluginlib: %s", e.what());
    }

    return std::unique_ptr<ReadableStorage>();
  }

private:
  std::unique_ptr<pluginlib::ClassLoader<WritableStorage>> writable_class_loader_;
  std::unique_ptr<pluginlib::ClassLoader<ReadableStorage>> readable_class_loader_;
};

}  // namespace rosbag2_storage

#endif  // IMPL__STORAGE_FACTORY_IMPL_HPP_

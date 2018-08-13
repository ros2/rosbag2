// Copyright 2018,  Open Source Robotics Foundation, Inc.
// Copyright 2018,  Bosch Software Innovations GmbH.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "rcutils/logging_macros.h"

#include "rosbag2_storage/readable_storage.hpp"
#include "rosbag2_storage/read_write_storage.hpp"
#include "rosbag2_storage/storage_factory.hpp"

namespace rosbag2_storage
{

const char * ROS_PACKAGE_NAME = "rosbag2_storage";

class StorageFactoryImpl
{
public:
  ~StorageFactoryImpl() = default;

  std::shared_ptr<ReadWriteStorage> get_read_write_storage(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      storage_loader_ = std::make_shared<pluginlib::ClassLoader<ReadWriteStorage>>(
        "rosbag2_storage", "rosbag2_storage::ReadWriteStorage");

      if (storage_id_is_present(storage_loader_->getDeclaredClasses(), storage_id)) {
        auto instance = storage_loader_->createUnmanagedInstance(storage_id);
        try {
          instance->open(uri);
        } catch (const std::runtime_error & ex) {
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
            "Could not open uri %s : %s", storage_id.c_str(), ex.what());
        }
        return std::shared_ptr<ReadWriteStorage>(instance);
      }

      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
        "Requested storage id %s does not exist", storage_id.c_str());
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error reading from pluginlib: %s", e.what());
    }

    return std::shared_ptr<ReadWriteStorage>();
  }

  std::shared_ptr<ReadableStorage> get_read_only_storage(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      readable_class_loader_ = std::make_shared<pluginlib::ClassLoader<ReadableStorage>>(
        "rosbag2_storage", "rosbag2_storage::ReadableStorage");

      if (storage_id_is_present(readable_class_loader_->getDeclaredClasses(), storage_id)) {
        auto instance = readable_class_loader_->createUnmanagedInstance(storage_id);
        try {
          instance->open_readonly(uri);
        } catch (const std::runtime_error & ex) {
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
            "Could not open uri %s : %s", storage_id.c_str(), ex.what());
        }
        return std::shared_ptr<ReadableStorage>(instance);
      }

      storage_loader_ = std::make_shared<pluginlib::ClassLoader<ReadWriteStorage>>(
        "rosbag2_storage", "rosbag2_storage::ReadWriteStorage");

      if (storage_id_is_present(storage_loader_->getDeclaredClasses(), storage_id)) {
        auto instance = storage_loader_->createUnmanagedInstance(storage_id);
        try {
          instance->open_readonly(uri);
        } catch (const std::runtime_error & ex) {
          RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
            "Could not open uri %s : %s", storage_id.c_str(), ex.what());
        }
        return std::shared_ptr<ReadableStorage>(instance);
      }

      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
        "Requested storage id %s does not exist", storage_id.c_str());
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error reading from pluginlib: %s", e.what());
    }

    return std::shared_ptr<ReadableStorage>();
  }

private:
  bool storage_id_is_present(
    std::vector<std::string> registered_classes, const std::string & storage_id)
  {
    return std::find(registered_classes.begin(), registered_classes.end(), storage_id) !=
           registered_classes.end();
  }

  std::shared_ptr<pluginlib::ClassLoader<ReadWriteStorage>> storage_loader_;
  std::shared_ptr<pluginlib::ClassLoader<ReadableStorage>> readable_class_loader_;
};

}  // namespace rosbag2_storage

#endif  // IMPL__STORAGE_FACTORY_IMPL_HPP_

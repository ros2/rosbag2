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

#include "rosbag2_storage/writable_storage.hpp"
#include "rosbag2_storage/readable_storage.hpp"
#include "rosbag2_storage/storage.hpp"
#include "rosbag2_storage/storage_factory.hpp"

namespace rosbag2_storage
{

const char * ROS_PACKAGE_NAME = "rosbag2_storage";

class StorageFactoryImpl
{
public:
  ~StorageFactoryImpl() = default;

  std::shared_ptr<Storage> get_storage(const std::string & storage_id, const std::string & uri)
  {
    try {
      storage_loader_ = std::make_shared<pluginlib::ClassLoader<Storage>>(
        "rosbag2_storage", "rosbag2_storage::Storage");

      if (storage_id_is_present(storage_loader_->getDeclaredClasses(), storage_id)) {
        return open_storage(storage_loader_, storage_id, uri);
      }

      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
        "Requested storage id %s does not exist", storage_id.c_str());
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error reading from pluginlib: %s", e.what());
    }

    return std::shared_ptr<Storage>();
  }

  std::shared_ptr<WritableStorage> get_write_only_storage(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      writable_class_loader_ = std::make_shared<pluginlib::ClassLoader<WritableStorage>>(
        "rosbag2_storage", "rosbag2_storage::WritableStorage");

      return open_storage_of_type<WritableStorage>(writable_class_loader_, storage_id, uri);
    } catch (std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME, "Error reading from pluginlib: %s", e.what());
    }

    return std::shared_ptr<WritableStorage>();
  }

  std::shared_ptr<ReadableStorage> get_read_only_storage(
    const std::string & storage_id, const std::string & uri)
  {
    try {
      readable_class_loader_ = std::make_shared<pluginlib::ClassLoader<ReadableStorage>>(
        "rosbag2_storage", "rosbag2_storage::ReadableStorage");

      return open_storage_of_type<ReadableStorage>(readable_class_loader_, storage_id, uri);
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

  template<typename T>
  std::shared_ptr<T> open_storage_of_type(
    std::shared_ptr<pluginlib::ClassLoader<T>> loader,
    const std::string & storage_id,
    const std::string & uri)
  {
    if (storage_id_is_present(loader->getDeclaredClasses(), storage_id)) {
      return open_storage(loader, storage_id, uri);
    }

    storage_loader_ = std::make_shared<pluginlib::ClassLoader<Storage>>(
      "rosbag2_storage", "rosbag2_storage::Storage");

    if (storage_id_is_present(storage_loader_->getDeclaredClasses(), storage_id)) {
      return open_storage_as_subtype<T>(storage_loader_, storage_id, uri);
    }

    RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
      "Requested storage id %s does not exist", storage_id.c_str());

    return std::shared_ptr<T>();
  }

  template<typename T>
  std::shared_ptr<T> open_storage(
    std::shared_ptr<pluginlib::ClassLoader<T>> storage_loader,
    std::string storage_id,
    std::string uri)
  {
    auto instance = storage_loader->createUnmanagedInstance(storage_id);
    instance->open(uri);
    return std::shared_ptr<T>(instance);
  }

  template<typename T>
  std::shared_ptr<T> open_storage_as_subtype(
    std::shared_ptr<pluginlib::ClassLoader<Storage>> storage_loader,
    std::string storage_id,
    std::string uri)
  {
    auto instance = storage_loader->createUnmanagedInstance(storage_id);
    instance->open(uri);
    return std::shared_ptr<T>(instance);
  }

  std::shared_ptr<pluginlib::ClassLoader<Storage>> storage_loader_;
  std::shared_ptr<pluginlib::ClassLoader<WritableStorage>> writable_class_loader_;
  std::shared_ptr<pluginlib::ClassLoader<ReadableStorage>> readable_class_loader_;
};

}  // namespace rosbag2_storage

#endif  // IMPL__STORAGE_FACTORY_IMPL_HPP_

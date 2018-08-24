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

#include "rosbag2_storage/storage_interfaces/read_only_interface.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "rosbag2_storage/storage_factory.hpp"
#include "rosbag2_storage/storage_traits.hpp"

namespace rosbag2_storage
{

constexpr const char * ROS_PACKAGE_NAME = "rosbag2_storage";

using storage_interfaces::ReadOnlyInterface;
using storage_interfaces::ReadWriteInterface;

template<typename InterfaceT>
std::shared_ptr<pluginlib::ClassLoader<InterfaceT>>
get_class_loader()
{
  const char * lookup_name = StorageTraits<InterfaceT>::name;
  return std::make_shared<pluginlib::ClassLoader<InterfaceT>>(ROS_PACKAGE_NAME, lookup_name);
}

template<
  typename InterfaceT,
  storage_interfaces::IOFlag flag = StorageTraits<InterfaceT>::io_flag
>
std::shared_ptr<InterfaceT>
get_interface_instance(
  std::shared_ptr<pluginlib::ClassLoader<InterfaceT>> class_loader,
  const std::string & storage_id,
  const std::string & uri,
  bool last_attempt)
{
  const auto & registered_classes = class_loader->getDeclaredClasses();
  auto class_exists = std::find(registered_classes.begin(), registered_classes.end(), storage_id);
  if (class_exists == registered_classes.end()) {
    // This prevents unnecessary error messages when loading a ReadWrite storage as ReadOnly storage
    if (last_attempt) {
      RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
        "Requested storage id %s does not exist", storage_id.c_str());
    }
    return nullptr;
  }

  std::shared_ptr<InterfaceT> instance = nullptr;
  try {
    auto unmanaged_instance = class_loader->createUnmanagedInstance(storage_id);
    instance = std::shared_ptr<InterfaceT>(unmanaged_instance);
  } catch (const std::runtime_error & ex) {
    RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
      "unable to load instance of read write interface: %s", ex.what());
    return nullptr;
  }

  try {
    instance->open(uri, flag);
    return instance;
  } catch (const std::runtime_error & ex) {
    RCUTILS_LOG_ERROR_NAMED(ROS_PACKAGE_NAME,
      "Could not open '%s' with %s. Error: %s", uri.c_str(), storage_id.c_str(), ex.what());
    return nullptr;
  }
}

class StorageFactoryImpl
{
public:
  StorageFactoryImpl()
  {
    try {
      read_write_class_loader_ = get_class_loader<ReadWriteInterface>();
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(
        ROS_PACKAGE_NAME, "unable to create class load instance: %s", e.what());
      throw e;
    }

    try {
      read_only_class_loader_ = get_class_loader<ReadOnlyInterface>();
    } catch (const std::exception & e) {
      RCUTILS_LOG_ERROR_NAMED(
        ROS_PACKAGE_NAME, "unable to create class load instance: %s", e.what());
      throw e;
    }
  }

  virtual ~StorageFactoryImpl() = default;

  std::shared_ptr<ReadWriteInterface> open_read_write(
    const std::string & uri, const std::string & storage_id)
  {
    return get_interface_instance(read_write_class_loader_, storage_id, uri, true);
  }

  std::shared_ptr<ReadOnlyInterface> open_read_only(
    const std::string & uri, const std::string & storage_id)
  {
    // try to load the instance as read_only interface
    auto instance = get_interface_instance(read_only_class_loader_, storage_id, uri, false);
    // try to load as read_write if not successful
    if (instance == nullptr) {
      instance = get_interface_instance<ReadWriteInterface, storage_interfaces::IOFlag::READ_ONLY>(
        read_write_class_loader_, storage_id, uri, true);
    }
    return instance;
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<ReadWriteInterface>> read_write_class_loader_;
  std::shared_ptr<pluginlib::ClassLoader<ReadOnlyInterface>> read_only_class_loader_;
};

}  // namespace rosbag2_storage

#endif  // IMPL__STORAGE_FACTORY_IMPL_HPP_

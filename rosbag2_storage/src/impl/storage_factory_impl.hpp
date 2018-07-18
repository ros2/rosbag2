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

namespace rosbag2_storage
{

class StorageFactoryImpl
{
public:
  explicit StorageFactoryImpl(const std::string & storage_identifier)
  : storage_identifier_(storage_identifier)
  {
    class_loader_ = std::make_unique<pluginlib::ClassLoader<rosbag2_storage::StorageInterface>>(
      "rosbag2_storage", "rosbag2_storage::StorageInterface");

    auto registered_classes = class_loader_->getDeclaredClasses();

    bool matched_id = false;
    fprintf(stderr, "registered classes\n");
    for (auto class_ : registered_classes) {
      fprintf(stderr, "%s\n", class_.c_str());
      if (class_ == storage_identifier) {
        fprintf(stderr, "%s\n", "found a matching class");
        matched_id = true;
      }
    }
    if (matched_id == false) {
      throw std::runtime_error("no matching class for storage id loaded");
    }
  }

  ~StorageFactoryImpl() {}

  std::shared_ptr<StorageInterface> get_instance()
  {
    auto instance = class_loader_->createSharedInstance(storage_identifier_);
    instance->set_storage_identifier(storage_identifier_);
    return instance;
  }

private:
  std::string storage_identifier_;
  std::unique_ptr<pluginlib::ClassLoader<rosbag2_storage::StorageInterface>> class_loader_;
};

}  // namespace rosbag2_storage

#endif  // IMPL__STORAGE_FACTORY_IMPL_HPP_

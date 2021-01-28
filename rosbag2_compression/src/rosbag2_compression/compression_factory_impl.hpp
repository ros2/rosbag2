// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_
#define ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "logging.hpp"
#include "rosbag2_compression/compression_factory.hpp"

namespace rosbag2_compression
{

using rosbag2_compression::BaseCompressorInterface;
using rosbag2_compression::BaseDecompressorInterface;

template<typename T>
struct CompressionTraits
{};

template<>
struct CompressionTraits<BaseCompressorInterface>
{
  static constexpr const char * name = "rosbag2_compression::BaseCompressorInterface";
};

template<>
struct CompressionTraits<BaseDecompressorInterface>
{
  static constexpr const char * name = "rosbag2_compression::BaseDecompressorInterface";
};

template<typename InterfaceT>
std::shared_ptr<pluginlib::ClassLoader<InterfaceT>>
get_class_loader()
{
  const char * lookup_name = CompressionTraits<InterfaceT>::name;
  return std::make_shared<pluginlib::ClassLoader<InterfaceT>>("rosbag2_compression", lookup_name);
}

template<typename InterfaceT>
std::unique_ptr<InterfaceT>
get_interface_instance(
  std::shared_ptr<pluginlib::ClassLoader<InterfaceT>> class_loader,
  const std::string & compression_format)
{
  const auto & registered_classes = class_loader->getDeclaredClasses();
  auto class_iter = std::find(
    registered_classes.begin(), registered_classes.end(), compression_format);
  if (class_iter == registered_classes.end()) {
    ROSBAG2_COMPRESSION_LOG_DEBUG_STREAM(
      "Requested compression format '" << compression_format << "' does not exist");
    return nullptr;
  }

  std::unique_ptr<InterfaceT> instance = nullptr;
  try {
    auto unmanaged_instance = class_loader->createUnmanagedInstance(compression_format);
    instance = std::unique_ptr<InterfaceT>(unmanaged_instance);
  } catch (const std::runtime_error & ex) {
    ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
      "Unable to load instance of compression interface: " << ex.what());
  }
  return instance;
}

/// Implementation of the CompressionFactory. See CompressionFactory for documentation.
class CompressionFactoryImpl
{
public:
  CompressionFactoryImpl()
  {
    try {
      compressor_class_loader_ = get_class_loader<BaseCompressorInterface>();
    } catch (const std::exception & e) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
      throw e;
    }

    try {
      decompressor_class_loader_ = get_class_loader<BaseDecompressorInterface>();
    } catch (const std::exception & e) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM("Unable to create class loader instance: " << e.what());
      throw e;
    }
  }

  virtual ~CompressionFactoryImpl() = default;

  /// See CompressionFactory::create_compressor for documentation.
  std::unique_ptr<rosbag2_compression::BaseCompressorInterface>
  create_compressor(const std::string & compression_format)
  {
    auto instance = get_interface_instance(compressor_class_loader_, compression_format);
    if (instance == nullptr) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
        "Could not load/open plugin for compression format '" << compression_format << "'");
    }
    return instance;
  }

  /// See CompressionFactory::create_decompressor for documentation.
  std::unique_ptr<rosbag2_compression::BaseDecompressorInterface>
  create_decompressor(const std::string & compression_format)
  {
    auto instance = get_interface_instance(decompressor_class_loader_, compression_format);
    if (instance == nullptr) {
      ROSBAG2_COMPRESSION_LOG_ERROR_STREAM(
        "Could not load/open plugin for compression format '" << compression_format << "'");
    }
    return instance;
  }

private:
  std::shared_ptr<pluginlib::ClassLoader<BaseCompressorInterface>> compressor_class_loader_;
  std::shared_ptr<pluginlib::ClassLoader<BaseDecompressorInterface>> decompressor_class_loader_;
};
}  // namespace rosbag2_compression

#endif  // ROSBAG2_COMPRESSION__COMPRESSION_FACTORY_IMPL_HPP_

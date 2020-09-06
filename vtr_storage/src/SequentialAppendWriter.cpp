#include "vtr_storage/SequentialAppendWriter.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <iostream>

#include "rcpputils/filesystem_helper.hpp"

#include "rosbag2_cpp/info.hpp"
#include "rosbag2_cpp/storage_options.hpp"

namespace vtr::storage
{

std::string format_storage_uri(const std::string & base_folder, uint64_t storage_count)
{
  // Right now `base_folder_` is always just the folder name for where to install the bagfile.
  // The name of the folder needs to be queried in case
  // SequentialWriter is opened with a relative path.
  std::stringstream storage_file_name;
  storage_file_name << rcpputils::fs::path(base_folder).filename().string() << "_" << storage_count;

  return (rcpputils::fs::path(base_folder) / storage_file_name.str()).string();
}


SequentialAppendWriter::SequentialAppendWriter(
  std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory,
  std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory,
  std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io)
: SequentialWriter(std::move(storage_factory), std::move(converter_factory), std::move(metadata_io))
{}

SequentialAppendWriter::~SequentialAppendWriter()
{

}

void SequentialAppendWriter::open(
  const rosbag2_cpp::StorageOptions & storage_options,
  const rosbag2_cpp::ConverterOptions & converter_options)
{
  base_folder_ = storage_options.uri;
  max_bagfile_size_ = storage_options.max_bagfile_size;
  max_bagfile_duration = std::chrono::seconds(storage_options.max_bagfile_duration);
  max_cache_size_ = storage_options.max_cache_size;

  cache_.reserve(max_cache_size_);

  if (converter_options.output_serialization_format !=
    converter_options.input_serialization_format)
  {
    converter_ = std::make_unique<rosbag2_cpp::Converter>(converter_options, converter_factory_);
  }
  rcpputils::fs::path db_path(base_folder_);
  if (db_path.is_directory()) {
    std::cout << "Database directory already exists (" << db_path.string() <<
      "), assuming append mode." << std::endl;
    append_mode_ = true;
  }
  bool dir_created = rcpputils::fs::create_directories(db_path); // will fail if file already exists, i.e. in append mode
  // if (!dir_created) {
  //   std::stringstream error;
  //   error << "Failed to create database directory (" << db_path.string() << ").";
  //   throw std::runtime_error{error.str()};
  // }

  const auto storage_uri = format_storage_uri(base_folder_, 0);

  storage_ = storage_factory_->open_read_write(storage_uri, storage_options.storage_id);
  if (!storage_) {
    throw std::runtime_error("No storage could be initialized. Abort");
  }
  if (max_bagfile_size_ != 0 &&
    max_bagfile_size_ < storage_->get_minimum_split_file_size())
  {
    std::stringstream error;
    error << "Invalid bag splitting size given. Please provide a value greater than " <<
      storage_->get_minimum_split_file_size() << ". Specified value of " <<
      storage_options.max_bagfile_size;
    throw std::runtime_error{error.str()};
  }
  if(append_mode_) {
    // get information on existing message topics, and add to the writer's metadata and converter
    // metadata_ = storage_->get_metadata();
    metadata_ = metadata_io_->read_metadata(storage_options.uri);
    std::cout << "Message count before appending: " << metadata_.message_count << std::endl;
    storage_->get_all_topics_and_types();
    for (const auto & topic_information : metadata_.topics_with_message_count) {
      
      const auto insert_res = topics_names_to_info_.insert(
        std::make_pair(topic_information.topic_metadata.name, topic_information));

      if (!insert_res.second) {
        std::stringstream errmsg;
        errmsg << "Failed to insert topic \"" << topic_information.topic_metadata.name << "\"!";

        throw std::runtime_error(errmsg.str());
      }
      if(converter_) {
        converter_->add_topic(topic_information.topic_metadata.name, topic_information.topic_metadata.type);
      }
    }
    std::cout << "Number of topics before appending: " << topics_names_to_info_.size() << std::endl;
  } else {
    init_metadata();
  }
}

}  // namespace vtr::storage

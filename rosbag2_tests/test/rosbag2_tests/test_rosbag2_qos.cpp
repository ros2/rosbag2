#include <gmock/gmock.h>

#include "rosbag2_transport/play_options.hpp"
#include "rosbag2_transport/storage_options.hpp"

namespace
{}

TEST() {
  rosbag2_transport::PlayOptions play_options{};
  play_options.node_prefix = "";
  play_options.read_ahead_queue_size = read_ahead_queue_size;

  rosbag2_transport::StorageOptions storage_options{};
  storage_options.uri = "";
  storage_options.storage_id = "";

  size_t read_ahead_queue_size;

  rosbag2_storage::MetadataIo metadata_io{};
  rosbag2_storage::BagMetadata metadata{};
  // Specify defaults
  auto info = std::make_shared<rosbag2_cpp::Info>();
  std::shared_ptr<rosbag2_cpp::Reader> reader;
  auto writer = std::make_shared<rosbag2_cpp::Writer>(
    std::make_unique<rosbag2_cpp::writers::SequentialWriter>());
  // Change reader based on metadata options
  if (metadata_io.metadata_file_exists(storage_options.uri)) {
    metadata = metadata_io.read_metadata(storage_options.uri);
    if (metadata.compression_format == "zstd") {
      reader = std::make_shared<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_compression::SequentialCompressionReader>());
    } else {
      reader = std::make_shared<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    }
  } else {
    reader = std::make_shared<rosbag2_cpp::Reader>(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  }

  rosbag2_transport::Rosbag2Transport transport(reader, writer, info);
  transport.init();
  transport.play(storage_options, play_options);
  transport.shutdown();
}

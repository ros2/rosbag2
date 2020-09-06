#ifndef VTR_STORAGE__SEQUENTIALAPPENDWRITER_HPP_
#define VTR_STORAGE__SEQUENTIALAPPENDWRITER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rosbag2_cpp/writers/sequential_writer.hpp"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace vtr::storage
{

class SequentialAppendWriter
  : public rosbag2_cpp::writers::SequentialWriter
{
public:
  explicit
  SequentialAppendWriter(
    std::unique_ptr<rosbag2_storage::StorageFactoryInterface> storage_factory =
    std::make_unique<rosbag2_storage::StorageFactory>(),
    std::shared_ptr<rosbag2_cpp::SerializationFormatConverterFactoryInterface> converter_factory =
    std::make_shared<rosbag2_cpp::SerializationFormatConverterFactory>(),
    std::unique_ptr<rosbag2_storage::MetadataIo> metadata_io =
    std::make_unique<rosbag2_storage::MetadataIo>());

  ~SequentialAppendWriter() override;

  /**
   * Opens a new bagfile and prepare it for writing messages if bagfile does not exist.
   * Opens the existing bagfile for appending messages if one exists.
   *
   * \param storage_options Options to configure the storage
   * \param converter_options options to define in which format incoming messages are stored
   **/
  void open(
    const rosbag2_cpp::StorageOptions & storage_options, const rosbag2_cpp::ConverterOptions & converter_options) override;

protected:
  bool append_mode_ = false;
};

}  // namespace vtr::storage

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // VTR_STORAGE__SEQUENTIALAPPENDWRITER_HPP_

// description
#ifndef VTR_STORAGE__DATASTREAMREADER_HPP_
#define VTR_STORAGE__DATASTREAMREADER_HPP_

#include "vtr_storage/DataStreamBase.hpp"
#include "vtr_storage/RandomAccessReader.hpp"

namespace vtr::storage
{

class DataStreamReader : public DataStreamBase
{
public:
    DataStreamReader(const std::string &data_directory,const std::string &stream_name);
    ~DataStreamReader();

    void openAndGetMessageType();
    void close();

    std::shared_ptr<TestMsgT> readAtIndex(int32_t index);
    std::shared_ptr<TestMsgT> readAtTimestamp(rcutils_time_point_value_t time);
    std::shared_ptr<std::vector<std::shared_ptr<TestMsgT>>> readAtIndexRange(int32_t index_begin,int32_t index_end);
    std::shared_ptr<std::vector<std::shared_ptr<TestMsgT>>> readAtTimestampRange(rcutils_time_point_value_t time_begin, rcutils_time_point_value_t time_end);

    // next() isn't needed for now due to ranged random access. Can add in if necessary

protected:
    std::shared_ptr<RandomAccessReader> reader_;
};


} // namespace vtr::storage
#endif // VTR_STORAGE__DATASTREAMREADER_HPP_
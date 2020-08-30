// description
#ifndef ROSBAG2_EXTENSIONS__DATASTREAMREADER_HPP_
#define ROSBAG2_EXTENSIONS__DATASTREAMREADER_HPP_

#include "rosbag2_extensions/DataStreamBase.hpp"

namespace rosbag2_extensions
{

class DataStreamReader : public DataStreamBase
{
public:
    DataStreamReader(const std::string &data_directory,const std::string &stream_name);
    ~DataStreamReader();

    void openAndGetMessageType();
    void close();

    std::shared_ptr<TestMsgT> readAtIndex(uint32_t index);
    std::shared_ptr<TestMsgT> readAtTimestamp(rcutils_time_point_value_t time);
    std::shared_ptr<std::vector<TestMsgT>> readAtIndexRange(uint32_t index_begin,uint32_t index_end);
    std::shared_ptr<std::vector<TestMsgT>> readAtTimestampRange(rcutils_time_point_value_t time_begin, rcutils_time_point_value_t time_end);

    // next() isn't needed for now due to ranged random access. Can add in if necessary

protected:
    rosbag2_cpp::readers::RandomAccessReader reader_;
};


}
#endif
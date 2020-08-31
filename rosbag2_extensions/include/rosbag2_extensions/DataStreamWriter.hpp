// description
#ifndef ROSBAG2_EXTENSIONS__DATASTREAMWRITER_HPP_
#define ROSBAG2_EXTENSIONS__DATASTREAMWRITER_HPP_

#include "rosbag2_extensions/DataStreamBase.hpp"

namespace rosbag2_extensions
{

class DataStreamWriter : public DataStreamBase
{
public:
    DataStreamWriter(const std::string &data_directory,const std::string &stream_name);
    ~DataStreamWriter();

    void open();
    void close();

    // returns the id of the inserted message
    int32_t write(const TestMsgT &message);

protected:
    rosbag2_storage::TopicMetadata createTopicMetadata();

    rosbag2_storage::TopicMetadata tm_;
    rosbag2_cpp::writers::SequentialWriter writer_;
};


}
#endif
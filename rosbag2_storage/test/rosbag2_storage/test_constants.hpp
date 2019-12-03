// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


#ifndef ROSBAG2_STORAGE__TEST_CONSTANTS_HPP_
#define ROSBAG2_STORAGE__TEST_CONSTANTS_HPP_

namespace test_constants
{
constexpr const char * const READ_WRITE_PLUGIN_IDENTIFIER = "ReadWritePlugin";
constexpr const char * const READ_ONLY_PLUGIN_IDENTIFIER = "ReadOnlyPlugin";
constexpr const char * const DUMMY_FILEPATH = "/path/to/storage";
constexpr const uint64_t MAX_BAGFILE_SIZE = 0;
constexpr const uint64_t MIN_SPLIT_FILE_SIZE = INT64_MAX;
}

#endif  // ROSBAG2_STORAGE__TEST_CONSTANTS_HPP_

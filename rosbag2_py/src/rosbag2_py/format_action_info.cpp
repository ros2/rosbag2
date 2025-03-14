// Copyright 2025 Sony Group Corporation.
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

#include <sstream>

#include "format_action_info.hpp"
#include "rosbag2_cpp/action_utils.hpp"

namespace rosbag2_py
{

std::string
format_action_info(
  std::vector<std::shared_ptr<rosbag2_cpp::rosbag2_action_info_t>> & action_info_list,
  const InfoSortingMethod sort_method)
{
  std::stringstream info_stream;
  const std::string action_info_string = "Action information: ";
  info_stream << "Actions:           " << action_info_list.size() << std::endl;
  info_stream << action_info_string;

  if (action_info_list.empty()) {
    return info_stream.str();
  }

  auto print_action_info =
    [&info_stream](const std::shared_ptr<rosbag2_cpp::rosbag2_action_info_t> & ai) -> void {
      info_stream << std::endl;
      info_stream << "  Action: " << ai->name << " | ";
      info_stream << "Type: " << ai->type << " | ";
      info_stream << "Topics: 2" << " | ";
      info_stream << "Service: 3" << " | ";
      info_stream << "Serialization Format: " << ai->serialization_format << std::endl;
      info_stream << "    Topic: feedback | Count: " << ai->feedback_topic_msg_count << std::endl;
      info_stream << "    Topic: status | Count: " << ai->status_topic_msg_count << std::endl;
      info_stream << "    Service: send_goal | Request Count: "
                  << ai->send_goal_service_msg_count.first
                  << " | Response Count: "
                  << ai->send_goal_service_msg_count.second << std::endl;
      info_stream << "    Service: cancel_goal | Request Count: "
                  << ai->cancel_goal_service_msg_count.first
                  << " | Response Count: "
                  << ai->cancel_goal_service_msg_count.second << std::endl;
      info_stream << "    Service: get_result | Request Count: "
                  << ai->get_result_service_msg_count.first
                  << " | Response Count: " << ai->get_result_service_msg_count.second;
    };

  std::vector<size_t> sorted_idx = generate_sorted_idx(action_info_list, sort_method);

  print_action_info(action_info_list[sorted_idx[0]]);
  auto number_of_services = action_info_list.size();
  for (size_t j = 1; j < number_of_services; ++j) {
    print_action_info(action_info_list[sorted_idx[j]]);
  }

  return info_stream.str();
}

}  // namespace rosbag2_py

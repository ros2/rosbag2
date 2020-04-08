# Copyright 2020 Amazon.com, Inc. or its affiliates. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import unittest

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from ros2bag.api import convert_yaml_to_qos_profile
from ros2bag.api import dict_to_duration
from ros2bag.api import interpret_dict_as_qos_profile


class TestRos2BagRecord(unittest.TestCase):

    def test_dict_to_duration_valid(self):
        expected_nanoseconds = 1000000002
        duration_dict = {'sec': 1, 'nsec': 2}
        duration = dict_to_duration(duration_dict)
        assert duration.nanoseconds == expected_nanoseconds

    def test_dict_to_duration_invalid(self):
        duration_dict = {'sec': 1}
        with self.assertRaises(ValueError):
            dict_to_duration(duration_dict)

    def test_interpret_dict_as_qos_profile_valid(self):
        qos_dict = {'history': 'keep_last', 'depth': 10}
        qos_profile = interpret_dict_as_qos_profile(qos_dict)
        assert qos_profile.history == QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
        expected_seconds = 1
        expected_nanoseconds = int((expected_seconds * 1e9))
        qos_dict = {'history': 'keep_all', 'deadline': {'sec': expected_seconds, 'nsec': 0}}
        qos_profile = interpret_dict_as_qos_profile(qos_dict)
        assert qos_profile.deadline.nanoseconds == expected_nanoseconds
        expected_convention = False
        qos_dict = {'history': 'keep_all', 'avoid_ros_namespace_conventions': expected_convention}
        qos_profile = interpret_dict_as_qos_profile(qos_dict)
        assert qos_profile.avoid_ros_namespace_conventions == expected_convention

    def test_interpret_dict_as_qos_profile_invalid(self):
        qos_dict = {'foo': 'bar'}
        with self.assertRaises(ValueError):
            interpret_dict_as_qos_profile(qos_dict)

    def test_convert_yaml_to_qos_profile(self):
        topic_name_1 = '/topic1'
        topic_name_2 = '/topic2'
        expected_convention = False
        qos_dict = {
            topic_name_1: {
                'history': 'keep_all', 'durability': 'volatile', 'reliability': 'reliable'},
            topic_name_2: {
                'history': 'keep_all', 'avoid_ros_namespace_conventions': expected_convention}
        }
        qos_profiles = convert_yaml_to_qos_profile(qos_dict)
        assert qos_profiles[topic_name_1].durability == \
            QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        assert qos_profiles[topic_name_1].reliability == \
            QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        assert qos_profiles[topic_name_1].history == \
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
        assert qos_profiles[topic_name_2].avoid_ros_namespace_conventions == expected_convention
        assert qos_profiles[topic_name_2].history == \
            QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL

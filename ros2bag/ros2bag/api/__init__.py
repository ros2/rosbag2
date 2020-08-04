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

from argparse import ArgumentTypeError
import os
from typing import Any
from typing import Dict
from typing import Optional

from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# This map needs to be updated when new policies are introduced
_QOS_POLICY_FROM_SHORT_NAME = {
    'history': QoSHistoryPolicy.get_from_short_key,
    'reliability': QoSReliabilityPolicy.get_from_short_key,
    'durability': QoSDurabilityPolicy.get_from_short_key,
    'liveliness': QoSLivelinessPolicy.get_from_short_key
}
_DURATION_KEYS = ['deadline', 'lifespan', 'liveliness_lease_duration']
_VALUE_KEYS = ['depth', 'avoid_ros_namespace_conventions']


def print_error(string: str) -> str:
    return '[ERROR] [ros2bag]: {}'.format(string)


def dict_to_duration(time_dict: Optional[Dict[str, int]]) -> Duration:
    """Convert a QoS duration profile from YAML into an rclpy Duration."""
    if time_dict:
        try:
            if (Duration(seconds=time_dict['sec'], nanoseconds=time_dict['nsec']) <
                    Duration(seconds=0)):
                raise ValueError('Time duration may not be a negative value.')
            return Duration(seconds=time_dict['sec'], nanoseconds=time_dict['nsec'])
        except KeyError:
            raise ValueError(
                'Time overrides must include both seconds (sec) and nanoseconds (nsec).')
    else:
        return Duration()


def interpret_dict_as_qos_profile(qos_profile_dict: Dict) -> QoSProfile:
    """Sanitize a user provided dict of a QoS profile and verify all keys are valid."""
    new_profile_dict = {}
    for policy_key, policy_value in qos_profile_dict.items():
        if policy_key in _DURATION_KEYS:
            new_profile_dict[policy_key] = dict_to_duration(policy_value)
        elif policy_key in _QOS_POLICY_FROM_SHORT_NAME:
            new_profile_dict[policy_key] = _QOS_POLICY_FROM_SHORT_NAME[policy_key](policy_value)
        elif policy_key in _VALUE_KEYS:
            if policy_value < 0:
                raise ValueError('`{}` may not be a negative value.'.format(policy_key))
            new_profile_dict[policy_key] = policy_value
        else:
            raise ValueError('Unexpected key `{}` for QoS profile.'.format(policy_key))
    return QoSProfile(**new_profile_dict)


def convert_yaml_to_qos_profile(qos_profile_dict: Dict) -> Dict[str, QoSProfile]:
    """Convert a YAML file to use rclpy's QoSProfile."""
    topic_profile_dict = {}
    for topic, profile in qos_profile_dict.items():
        topic_profile_dict[topic] = interpret_dict_as_qos_profile(profile)
    return topic_profile_dict


def create_bag_directory(uri: str) -> Optional[str]:
    """Create a directory."""
    try:
        os.makedirs(uri)
    except OSError:
        return print_error("Could not create bag folder '{}'.".format(uri))


def check_positive_float(value: Any) -> float:
    """Argparse validator to verify that a value is a float and positive."""
    try:
        fvalue = float(value)
        if fvalue <= 0.0:
            raise ArgumentTypeError('{} is not in the valid range (> 0.0)'.format(value))
        return fvalue
    except ValueError:
        raise ArgumentTypeError('{} is not the valid type (float)'.format(value))


def check_path_exists(value: Any) -> str:
    """Argparse validator to verify a path exists."""
    try:
        if os.path.exists(value):
            return value
        raise ArgumentTypeError("Bag file '{}' does not exist!".format(value))
    except ValueError:
        raise ArgumentTypeError('{} is not the valid type (string)'.format(value))

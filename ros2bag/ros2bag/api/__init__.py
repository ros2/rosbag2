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
from typing import Dict
from typing import Optional

from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

# This map needs to be updated when new policies are introduced
POLICY_MAP = {
    'history': QoSHistoryPolicy.get_from_short_key,
    'reliability': QoSReliabilityPolicy.get_from_short_key,
    'durability': QoSDurabilityPolicy.get_from_short_key,
    'liveliness': QoSLivelinessPolicy.get_from_short_key
}


def is_dict_valid_duration(duration_dict: Dict[str, int]) -> bool:
    """Check if dictionary contains the keys `sec` and `nsec`."""
    return all(key in duration_dict for key in ['sec', 'nsec'])


def dict_to_duration(time_dict: Optional[Dict[str, int]]) -> Duration:
    """Convert a QoS duration profile from YAML into an rclpy Duration."""
    if time_dict:
        if is_dict_valid_duration(time_dict):
            return Duration(seconds=time_dict.get('sec'), nanoseconds=time_dict.get('nsec'))
        else:
            raise ValueError(
                'Time overrides must include both seconds (sec) and nanoseconds (nsec).')
    else:
        return Duration()


def validate_qos_profile_overrides(qos_profile_dict: Dict) -> Dict[str, Dict]:
    """Validate the QoS profile yaml file path and its structure."""
    for name in qos_profile_dict.keys():
        profile = qos_profile_dict[name]
        # Convert dict to Duration. Required for construction
        conversion_keys = ['deadline', 'lifespan', 'liveliness_lease_duration']
        for k in conversion_keys:
            profile[k] = dict_to_duration(profile.get(k))
        for policy in POLICY_MAP.keys():
            profile[policy] = POLICY_MAP[policy](profile.get(policy, 'system_default'))
        qos_profile_dict[name] = QoSProfile(**profile)
    return qos_profile_dict


def create_bag_directory(uri: str) -> str:
    """Create a directory."""
    try:
        os.makedirs(uri)
    except OSError:
        return "[ERROR] [ros2bag]: Could not create bag folder '{}'.".format(uri)


def check_positive_float(value: float) -> float:
    """Verify value a float and positive."""
    try:
        fvalue = float(value)
        if fvalue <= 0.0:
            raise ArgumentTypeError('%s is not in the valid range (> 0.0)' % value)
        return fvalue
    except ValueError:
        raise ArgumentTypeError('%s is not of the valid type (float)' % value)

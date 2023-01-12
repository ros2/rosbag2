# Copyright 2023 Foxglove Technologies Inc. All Rights Reserved.
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

from rosbag2_py import get_registered_compressors


class BagCommandExtension():

    def get_compression_modes(self):
        return ['none', 'message', 'file']

    def get_compression_formats(self):
        return list(get_registered_compressors())

    def get_preset_profiles(self):
        return ['none', 'resilient']

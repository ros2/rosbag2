# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import importlib
import os


def _import(name):
    # New in Python 3.8: on Windows we should call 'add_dll_directory()' for directories
    # containing DLLs we depend on.
    # https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
    dll_dir_handles = []
    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        path_env = os.environ['PATH'].split(';')
        for prefix_path in path_env:
            if os.path.exists(prefix_path):
                dll_dir_handles.append(os.add_dll_directory(prefix_path))
    try:
        return importlib.import_module(name, package='rosbag2_transport')
    except ImportError as e:
        if e.path is not None and os.path.isfile(e.path):
            e.msg += \
                "\nThe C extension '%s' failed to be imported while being present on the system." \
                " Please refer to '%s' for possible solutions" % \
                (e.path, 'https://index.ros.org/doc/ros2/Troubleshooting/'
                         '#import-failing-even-with-library-present-on-the-system')
        raise
    finally:
        for handle in dll_dir_handles:
            handle.close()


rosbag2_transport_py = _import('._rosbag2_transport_py')

# rosbag2_py

## Regenerating Python stub files (.pyi)

Python stub files allow to supply type-hinting information for binary Python modules (e.g. pybind-based).

In rosbag2_py stub files are generated with utility called `stubgen`.

To regenerate stub files
```
cd <workspace>
colcon build --packages-up-to rosbag2_py
source install/setup.sh
# Make sure rosbag2_py can be imported
python3 -c 'import rosbag2_py'

# Ubuntu 24.04
sudo apt update && sudo apt install mypy

# Older Ubuntu
# pip3 install -U mypy==1.9

cd <rosbag2 git repo>
stubgen -p rosbag2_py -o rosbag2_py
```

# Rosbag2 performance

## Building

1. Source ROS foxy
2. Build like standard ROS2 package:

    ```bash
    colcon build --symlink-install
    ```

## Usage

Temporarily there are only some bash scripts (in `rosbag2_benchmarking` dir) for testing how much images `rosbag2` is capable of writing depends on `--max-cache-size` parameter.

**Scripts:**

* `rosbag_image_cache.sh X` - run rosbag record on only `/image` topic with `--max-cache-size` set to `X`,
* `gen_images.sh X Y Z` - generate and publish `X` random images every `Y` ms of size `ZxZ`,
* `dummy_raport.sh` - shows how many images have been generated and how many `rosbag2` succeeded to write.

**Procedure:**

1. Run `rosbag_image_cache.sh X` to set up `rosbag2`,
2. Run `gen_images.sh X Y Z` to feed images to `rosbag2`,
3. After image generation, kill `rosbag_image_cache.sh` and see raport with `dummy_raport.sh`.
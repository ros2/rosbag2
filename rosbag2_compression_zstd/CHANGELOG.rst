^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_compression_zstd
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2022-07-30)
-------------------
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`_)
 * Update compression to make copy instead of in-place operation
 * Get rid of extra data copying operation in writer by refrencing to received message
* Contributors: DensoADAS, Joshua Hampp

0.16.0 (2022-05-11)
-------------------

0.15.1 (2022-04-06)
-------------------

0.15.0 (2022-04-05)
-------------------

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Install headers to include/${PROJECT_NAME} (`#958 <https://github.com/ros2/rosbag2/issues/958>`_)
* Contributors: Shane Loretz

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Contributors: Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------

0.7.0 (2021-03-18)
------------------
* Add test_depend ament_cmake_gmock (`#639 <https://github.com/ros2/rosbag2/issues/639>`_)
* Move zstd compressor to its own package (`#636 <https://github.com/ros2/rosbag2/issues/636>`_)
* Contributors: Emerson Knapp, Shane Loretz

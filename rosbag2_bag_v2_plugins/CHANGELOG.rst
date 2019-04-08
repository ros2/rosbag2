^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_bag_v2_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2018-12-14)
------------------
* Release fixes (`#73 <https://github.com/ros2/rosbag2/issues/73>`_)
* Play old bagfiles (`#69 <https://github.com/ros2/rosbag2/issues/69>`_)
* Contributors: Karsten Knese, Martin Idel

0.0.7 (2019-04-08)
------------------
* [backport] ros1 dependency handling (`#98 <https://github.com/ros2/rosbag2/issues/98>`_)
  * removed dependency to ros1_bridge package (`#90 <https://github.com/ros2/rosbag2/issues/90>`_)
  * removed dependency to ros1_bridge package:
  * checking if package is available
  * if not skipping (with warnings)
  * now rosbag2_tests builds on systems without ros1
  * check ros1 deps correctly on all packages
  * add ros1_bridge to test package
  * silently try to find the bridge
  * correct missing linter errors (`#96 <https://github.com/ros2/rosbag2/issues/96>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.0.6 (2019-02-27)
------------------

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
* 0.0.3
* manually bump version number as catkin_prepare_release complains
* generate changelog
* Contributors: Karsten Knese, Martin Idel

0.0.2 (2018-12-12)
------------------

0.0.1 (2018-12-11)
------------------

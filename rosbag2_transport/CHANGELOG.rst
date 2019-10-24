^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



0.2.1 (2019-10-23)
------------------
* Disable parameter event publishers on test nodes. (`#180 <https://github.com/ros2/rosbag2/issues/180>`_)
* Narrow down tests for topic discovery. (`#178 <https://github.com/ros2/rosbag2/issues/178>`_)
* Fix API for new Intra-Process communication. (`#143 <https://github.com/ros2/rosbag2/issues/143>`_)
* Add dependency on python_cmake_module. (`#188 <https://github.com/ros2/rosbag2/issues/188>`_)
* Add bagfile splitting support to storage_options. (`#182 <https://github.com/ros2/rosbag2/issues/182>`_)
* Fix the test failure of wrong messages count. (`#165 <https://github.com/ros2/rosbag2/issues/165>`_)
* Support for zero-copy message transport. (`#168 <https://github.com/ros2/rosbag2/issues/168>`_)
* Contributors: Alberto Soragna, ChenYing Kuo, Dan Rose, Karsten Knese, Mikael Arguedas, Zachary Michaels

0.2.0 (2019-09-26)
------------------
* fixup after API changes to Subscription in rclcpp (`#166 <https://github.com/ros2/rosbag2/issues/166>`_)
* disable some tests for connext (`#145 <https://github.com/ros2/rosbag2/issues/145>`_)
* disable plugins/tests which need rmw_fastrtps_cpp if unavailable (`#137 <https://github.com/ros2/rosbag2/issues/137>`_)
* Fix test failures on armhf (`#135 <https://github.com/ros2/rosbag2/issues/135>`_)
* Contributors: Karsten Knese, Prajakta Gokhale, William Woodall, ivanpauno

0.1.2 (2019-05-20)
------------------
* Fixes an init race condition (`#93 <https://github.com/ros2/rosbag2/issues/93>`_)
  * This could probably be a race condition, for ex: When we've create a subscriber in the API, and the subscriber has the data already available in the callback (Cause of existing publishers) the db entry for the particular topic would not be availalble, which in turn returns an SqliteException. This is cause write\_->create_topic() call is where we add the db entry for a particular topic. And, this leads to crashing before any recording.
  Locally I solved it by adding the db entry first, and if
  create_subscription fails, remove the topic entry from the db and also
  erase the subscription.
  Signed-off-by: Sriram Raghunathan <rsriram7@visteon.com>
  * Fix comments for pull request https://github.com/ros2/rosbag2/pull/93
  Signed-off-by: Sriram Raghunathan <rsriram7@visteon.com>
  * Added unit test case for remove_topics from db
  Signed-off-by: Sriram Raghunathan <rsriram7@visteon.com>
  * Fix unit tests failing by adding dependent test macros
  Signed-off-by: Sriram Raghunathan <rsriram7@visteon.com>
  * Fixes the linter errors
* Update troubleshooting reference to index.ros.org (`#120 <https://github.com/ros2/rosbag2/issues/120>`_)
  Signed-off-by: Michael Carroll <michael@openrobotics.org>
* Contributors: Michael Carroll, Sriram Raghunathan

0.1.1 (2019-05-09)
------------------
* fix condition to only apply pragma for GCC 8+ (`#117 <https://github.com/ros2/rosbag2/issues/117>`_)
* Contributors: Dirk Thomas

0.1.0 (2019-05-08)
------------------
* ignore cast function type warning (`#116 <https://github.com/ros2/rosbag2/issues/116>`_)
* changes to avoid deprecated API's (`#115 <https://github.com/ros2/rosbag2/issues/115>`_)
* Handle message type name with multiple namespace parts (`#114 <https://github.com/ros2/rosbag2/issues/114>`_)
* fix compilation against master (`#111 <https://github.com/ros2/rosbag2/issues/111>`_)
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
* use fastrtps static instead of dynamic (`#104 <https://github.com/ros2/rosbag2/issues/104>`_)
* enforce unique node names (`#86 <https://github.com/ros2/rosbag2/issues/86>`_)
* disable cppcheck (`#91 <https://github.com/ros2/rosbag2/issues/91>`_)
* Consistent node naming across ros2cli tools (`#60 <https://github.com/ros2/rosbag2/issues/60>`_)
* Contributors: AAlon, Dirk Thomas, Jacob Perron, Karsten Knese, William Woodall

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
* Improve queue usage (`#75 <https://github.com/bsinno/rosbag2/issues/75>`_)
* 0.0.3
* Play old bagfiles (`#69 <https://github.com/bsinno/rosbag2/issues/69>`_)
* Release fixups (`#72 <https://github.com/bsinno/rosbag2/issues/72>`_)
* Contributors: Andreas Holzner, Karsten Knese, Martin Idel

0.0.2 (2018-12-12)
------------------
* update maintainer email
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* Auto discovery of new topics (`#63 <https://github.com/ros2/rosbag2/issues/63>`_)
* Fix master build and small renamings (`#67 <https://github.com/ros2/rosbag2/issues/67>`_)
* rename topic_with_types to topic_metadata
* use converter options
* iterate_over_formatter
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* use uint8 for serialized message (`#61 <https://github.com/ros2/rosbag2/issues/61>`_)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
* Use converters when playing back files (`#56 <https://github.com/ros2/rosbag2/issues/56>`_)
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* GH-117 Check also for rclcpp::ok() when playing back messages (`#54 <https://github.com/ros2/rosbag2/issues/54>`_)
* Extract recorder from rosbag2_transport, fix test naming (`#44 <https://github.com/ros2/rosbag2/issues/44>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel

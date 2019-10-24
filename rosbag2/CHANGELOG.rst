^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Forthcoming
-----------
* Add get_identifier to base io-interfaces for support in bagfile splitting (`#183 <https://github.com/ros2/rosbag2/issues/183>`_)
* Add bagfile splitting support to storage_options (`#182 <https://github.com/ros2/rosbag2/issues/182>`_)
* Support for zero copy API (`#168 <https://github.com/ros2/rosbag2/issues/168>`_)
* Change storage interfaces for bagfile splitting feature (`#170 <https://github.com/ros2/rosbag2/issues/170>`_)
* Contributors: Karsten Knese, Zachary Michaels

0.2.0 (2019-09-26)
------------------
* enable address sanitizers only on 64bit machines (`#149 <https://github.com/ros2/rosbag2/issues/149>`_)
* Export pluginlib to downstream packages (`#113 <https://github.com/ros2/rosbag2/issues/113>`_)
* Add support for parsing middle module name from type (`#128 <https://github.com/ros2/rosbag2/issues/128>`_)
* Contributors: David Hodo, Esteve Fernandez, Karsten Knese

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
* Contributors: Sriram Raghunathan

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* Handle message type name with multiple namespace parts (`#114 <https://github.com/ros2/rosbag2/issues/114>`_)
* fix compilation against master (`#111 <https://github.com/ros2/rosbag2/issues/111>`_)
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
* Compile tests (`#103 <https://github.com/ros2/rosbag2/issues/103>`_)
* Contributors: Dirk Thomas, Jacob Perron, Karsten Knese

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
* 0.0.3
* Play old bagfiles (`#69 <https://github.com/bsinno/rosbag2/issues/69>`_)
* Contributors: Karsten Knese, Martin Idel

0.0.2 (2018-12-12)
------------------
* update maintainer email
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* Split converters (`#70 <https://github.com/ros2/rosbag2/issues/70>`_)
* GH-144 Add missing pop for warning pragma (`#68 <https://github.com/ros2/rosbag2/issues/68>`_)
* Fix master build and small renamings (`#67 <https://github.com/ros2/rosbag2/issues/67>`_)
* rename topic_with_types to topic_metadata
* use converter options
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
* Use converters when playing back files (`#56 <https://github.com/ros2/rosbag2/issues/56>`_)
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* Add entry point for converter plugins (`#47 <https://github.com/ros2/rosbag2/issues/47>`_)
* Extract recorder from rosbag2_transport, fix test naming (`#44 <https://github.com/ros2/rosbag2/issues/44>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* Add correct timing behaviour for rosbag play (`#32 <https://github.com/ros2/rosbag2/issues/32>`_)
* Improve sqlite usage and test stability (`#31 <https://github.com/ros2/rosbag2/issues/31>`_)
* Record and play multiple topics (`#27 <https://github.com/ros2/rosbag2/issues/27>`_)
* Allow an arbitrary topic to be recorded (`#26 <https://github.com/ros2/rosbag2/issues/26>`_)
* Use serialized message directly (`#24 <https://github.com/ros2/rosbag2/issues/24>`_)
* initial version of plugin based storage api (`#7 <https://github.com/ros2/rosbag2/issues/7>`_)
* add visibility macro (`#22 <https://github.com/ros2/rosbag2/issues/22>`_)
* (demo, sqlite3) First working rosbag2 implementation (`#6 <https://github.com/ros2/rosbag2/issues/6>`_)
* initial setup
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel

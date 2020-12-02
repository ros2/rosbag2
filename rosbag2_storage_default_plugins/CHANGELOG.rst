^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage_default_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Forthcoming
-----------

0.4.0 (2020-11-19)
------------------
* read yaml config file (`#497 <https://github.com/ros2/rosbag2/issues/497>`_)
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Contributors: Karsten Knese, Michael Jeronimo

0.3.2 (2020-06-03)
------------------

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------

0.2.5 (2020-04-30)
------------------
* Add filter for reading selective topics (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* Replace rcutils_get_file_size with rcpputils::fs::file_size (`#291 <https://github.com/ros2/rosbag2/issues/291>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Improve SQLite error messages (`#269 <https://github.com/ros2/rosbag2/issues/269>`_)
* remove rosbag2 filesystem helper (`#249 <https://github.com/ros2/rosbag2/issues/249>`_)
* Sanitize bagfile splitting CLI input (`#226 <https://github.com/ros2/rosbag2/issues/226>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Prajakta Gokhale, Sriram Raghunathan, Zachary Michaels

0.2.4 (2019-11-18)
------------------

0.2.3 (2019-11-18)
------------------
* Enhance rosbag writer capabilities to split bag files. (`#185 <https://github.com/ros2/rosbag2/issues/185>`_)
* Contributors: Zachary Michaels

0.2.2 (2019-11-13)
------------------
* (API) Generate bagfile metadata in Writer (`#184 <https://github.com/ros2/rosbag2/issues/184>`_)
* Contributors: Zachary Michaels

0.2.1 (2019-10-23)
------------------
* Add get_identifier to storage io-interface for support in bagfile splitting. (`#183 <https://github.com/ros2/rosbag2/issues/183>`_)
* Change storage interfaces for bagfile splitting feature (`#170 <https://github.com/ros2/rosbag2/issues/170>`_)
* Add error checking on SqliteWrapper deconstructor. (`#169 <https://github.com/ros2/rosbag2/issues/169>`_)
* Contributors: Zachary Michaels

0.2.0 (2019-09-26)
------------------

0.1.2 (2019-05-20)
------------------
* Indexing of messages.timestamp to avoid runtime-copy. (`#121 <https://github.com/ros2/rosbag2/issues/121>`_)
  Extended SqliteStorage::initialize() to add an index for the message table's timestamp column.
  Without this, the ORDER BY query in prepare_for_reading() causes expensive table duplication,
  which also has potential for out-of-disk or out-of-memory errors.
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
* Contributors: Felix-El, Sriram Raghunathan

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* fix line length of logging macros (`#110 <https://github.com/ros2/rosbag2/issues/110>`_)
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
* Contributors: Dirk Thomas, Karsten Knese

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
* fix unused variable warning when in release
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* rename topic_with_types to topic_metadata
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* use uint8 for serialized message (`#61 <https://github.com/ros2/rosbag2/issues/61>`_)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* Use directory as bagfile and add additonal record options (`#43 <https://github.com/ros2/rosbag2/issues/43>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* Add correct timing behaviour for rosbag play (`#32 <https://github.com/ros2/rosbag2/issues/32>`_)
* Improve sqlite iterator interface (`#33 <https://github.com/ros2/rosbag2/issues/33>`_)
* Improve sqlite usage and test stability (`#31 <https://github.com/ros2/rosbag2/issues/31>`_)
* Record all topics (`#30 <https://github.com/ros2/rosbag2/issues/30>`_)
* Record and play multiple topics (`#27 <https://github.com/ros2/rosbag2/issues/27>`_)
* Allow an arbitrary topic to be recorded (`#26 <https://github.com/ros2/rosbag2/issues/26>`_)
* Use serialized message directly (`#24 <https://github.com/ros2/rosbag2/issues/24>`_)
* add visibility macros (`#28 <https://github.com/ros2/rosbag2/issues/28>`_)
* initial version of plugin based storage api (`#7 <https://github.com/ros2/rosbag2/issues/7>`_)
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel

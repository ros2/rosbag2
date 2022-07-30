^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2022-07-30)
-------------------
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`_)
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`_)
 * Update compression to make copy instead of in-place operation
 * Get rid of extra data copying operation in writer by refrencing to received message
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`_)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`_)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)
* Contributors: DensoADAS, Joshua Hampp, Esteve Fernandez, Hunter L. Allen, Michael Orlov,
  Tony Peng

0.16.0 (2022-05-11)
-------------------

0.15.1 (2022-04-06)
-------------------
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Audrow Nash, Jorge Perez, Tony Peng

0.15.0 (2022-04-05)
-------------------
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Jorge Perez, Tony Peng

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Install headers to include/${PROJECT_NAME} (`#958 <https://github.com/ros2/rosbag2/issues/958>`_)
* Remove unnecessary public definition. (`#950 <https://github.com/ros2/rosbag2/issues/950>`_)
* Contributors: Chris Lalancette, Shane Loretz

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------
* Enable YAML encoding/decoding for RecordOptions and StorageOptions (`#916 <https://github.com/ros2/rosbag2/issues/916>`_)
* Contributors: Emerson Knapp

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Provide MetadataIO interface to convert metadata to a string in memory, alongside file IO versions (`#894 <https://github.com/ros2/rosbag2/issues/894>`_)
* Contributors: Emerson Knapp, Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Metadata per file info (`#870 <https://github.com/ros2/rosbag2/issues/870>`_)
* Implement snapshot mechanism and corresponding ROS Service (`#850 <https://github.com/ros2/rosbag2/issues/850>`_)
* added seek interface (`#836 <https://github.com/ros2/rosbag2/issues/836>`_)
* Refactor plugin query mechanism and standardize trait management (`#833 <https://github.com/ros2/rosbag2/issues/833>`_)
* Contributors: Cameron Miller, Wojciech Jaworski, sonia

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* PlayerClock initial implementation - Player functionally unchanged (`#689 <https://github.com/ros2/rosbag2/issues/689>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Reindexer core (`#641 <https://github.com/ros2/rosbag2/issues/641>`_)
  Add a new C++ Reindexer class for reconstructing metadata from bags that are missing it.
* Contributors: Emerson Knapp, jhdcs

0.7.0 (2021-03-18)
------------------
* Remove outdated pluginlib cmake script from rosbag2_storage (`#661 <https://github.com/ros2/rosbag2/issues/661>`_)
* CLI query rosbag2_py for available storage implementations (`#659 <https://github.com/ros2/rosbag2/issues/659>`_)
* Shorten some excessively long lines of CMake (`#648 <https://github.com/ros2/rosbag2/issues/648>`_)
* Contributors: Emerson Knapp, Scott K Logan

0.6.0 (2021-02-01)
------------------
* SQLite storage optimized by default (`#568 <https://github.com/ros2/rosbag2/issues/568>`_)
  * Use optimized pragmas by default in sqlite storage. Added option to use former behavior
* Use std::filesystem compliant non-member `exists` function for path object (`#593 <https://github.com/ros2/rosbag2/issues/593>`_)
* Contributors: Adam Dąbrowski, Josh Langsfeld

0.5.0 (2020-12-02)
------------------
* Update codes since rcutils_calculate_directory_size() is changed (`#567 <https://github.com/ros2/rosbag2/issues/567>`_)
* Contributors: Barry Xu

0.4.0 (2020-11-19)
------------------
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Add split by time to recording (`#409 <https://github.com/ros2/rosbag2/issues/409>`_)
* Contributors: Karsten Knese, Michael Jeronimo, jhdcs

0.3.2 (2020-06-03)
------------------

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Export targets (`#403 <https://github.com/ros2/rosbag2/issues/403>`_)
* Contributors: Karsten Knese

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------
* Correct usage of rclcpp::SharedLibrary loading. (`#400 <https://github.com/ros2/rosbag2/issues/400>`_)
* Contributors: Karsten Knese

0.2.5 (2020-04-30)
------------------
* Read serialized qos profiles out of the metadata (`#359 <https://github.com/ros2/rosbag2/issues/359>`_)
* Add filter for reading selective topics (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* remove rosbag2 filesystem helper (`#249 <https://github.com/ros2/rosbag2/issues/249>`_)
* [Compression - 7] Add compression metadata (`#221 <https://github.com/ros2/rosbag2/issues/221>`_)
* Sanitize bagfile splitting CLI input (`#226 <https://github.com/ros2/rosbag2/issues/226>`_)
* Move get_storage_identifier and get_bagfile_size (`#209 <https://github.com/ros2/rosbag2/issues/209>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Prajakta Gokhale, Sriram Raghunathan, Zachary Michaels

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
* Add get_identifier to base io-interfaces for support in bagfile splitting (`#183 <https://github.com/ros2/rosbag2/issues/183>`_)
* Add bagfile splitting support to storage_options (`#182 <https://github.com/ros2/rosbag2/issues/182>`_)
* Change storage interfaces for bagfile splitting feature (`#170 <https://github.com/ros2/rosbag2/issues/170>`_)
* Contributors: Zachary Michaels

0.2.0 (2019-09-26)
------------------
* Fix test failures on armhf (`#135 <https://github.com/ros2/rosbag2/issues/135>`_)
* Export pluginlib to downstream packages (`#113 <https://github.com/ros2/rosbag2/issues/113>`_)
* Contributors: Esteve Fernandez, Prajakta Gokhale

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
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
* Contributors: Dirk Thomas

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
* rename topic_with_types to topic_metadata
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* use uint8 for serialized message (`#61 <https://github.com/ros2/rosbag2/issues/61>`_)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
* Use converters when playing back files (`#56 <https://github.com/ros2/rosbag2/issues/56>`_)
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* Use directory as bagfile and add additonal record options (`#43 <https://github.com/ros2/rosbag2/issues/43>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* Add correct timing behaviour for rosbag play (`#32 <https://github.com/ros2/rosbag2/issues/32>`_)
* Improve sqlite usage and test stability (`#31 <https://github.com/ros2/rosbag2/issues/31>`_)
* Record and play multiple topics (`#27 <https://github.com/ros2/rosbag2/issues/27>`_)
* Allow an arbitrary topic to be recorded (`#26 <https://github.com/ros2/rosbag2/issues/26>`_)
* Use serialized message directly (`#24 <https://github.com/ros2/rosbag2/issues/24>`_)
* initial version of plugin based storage api (`#7 <https://github.com/ros2/rosbag2/issues/7>`_)
* (demo, sqlite3) First working rosbag2 implementation (`#6 <https://github.com/ros2/rosbag2/issues/6>`_)
* initial setup
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel

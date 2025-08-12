^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_storage_default_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.9 (2025-08-11)
-------------------

0.26.8 (2025-07-10)
-------------------
* Bugfix: Undefined behavior in the `rosbag2_storage` and `rosbag2_storage_sqlite3` packages (`#1997 <https://github.com/ros2/rosbag2/issues/1997>`_) (`#1999 <https://github.com/ros2/rosbag2/issues/1999>`_)
* [jazzy] Upstream quality changes from Apex.AI part-2 (backport `#1924 <https://github.com/ros2/rosbag2/issues/1924>`_) (`#1987 <https://github.com/ros2/rosbag2/issues/1987>`_)
* Contributors: Michael Orlov <morlovmr@gmail.com>, Christophe Bedard <bedard.christophe@gmail.com>

0.26.7 (2025-04-22)
-------------------

0.26.6 (2024-12-18)
-------------------

0.26.5 (2024-09-06)
-------------------
* Fix incorrect zero size for sqlite storage (`#1759 <https://github.com/ros2/rosbag2/issues/1759>`_) (`#1761 <https://github.com/ros2/rosbag2/issues/1761>`_)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
  (cherry picked from commit 86f681d9f525a4b7c3b4133d6b657e342283aad8)
  Co-authored-by: Roman <rsokolkov@gmail.com>
* Fix for failing throws_on_invalid_pragma_in_config_file on Windows (`#1742 <https://github.com/ros2/rosbag2/issues/1742>`_) (`#1746 <https://github.com/ros2/rosbag2/issues/1746>`_)
  (cherry picked from commit 055935d33dd2fed2772657c9dc1f2173eaa7f752)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
* Contributors: mergify[bot]

0.26.4 (2024-06-27)
-------------------
* Add topics with zero message counts to the SQLiteStorage::get_metadata(). (`#1725 <https://github.com/ros2/rosbag2/issues/1725>`_) (`#1731 <https://github.com/ros2/rosbag2/issues/1731>`_)
  Co-authored-by: Michael Orlov <michael.orlov@apex.ai>
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* Contributors: mergify[bot]

0.26.3 (2024-05-15)
-------------------

0.26.2 (2024-04-24)
-------------------

0.26.1 (2024-04-17)
-------------------

0.26.0 (2024-04-16)
-------------------
* Support service 2/2 --- rosbag2 service play (`#1481 <https://github.com/ros2/rosbag2/issues/1481>`_)
* Use middleware send and receive timestamps from message_info during recording (`#1531 <https://github.com/ros2/rosbag2/issues/1531>`_)
* Update to use yaml-cpp version 0.8.0. (`#1605 <https://github.com/ros2/rosbag2/issues/1605>`_)
* Contributors: Barry Xu, Chris Lalancette, jmachowinski, Michael Orlov

0.25.0 (2024-03-27)
-------------------
* Make some changes for newer versions of uncrustify. (`#1578 <https://github.com/ros2/rosbag2/issues/1578>`_)
* Add topic_id returned by storage to the TopicMetadata (`#1538 <https://github.com/ros2/rosbag2/issues/1538>`_)
* Remove rcpputils::fs dependencies from rosbag2_storages (`#1558 <https://github.com/ros2/rosbag2/issues/1558>`_)
* Change an incorrect TSA annotation. (`#1552 <https://github.com/ros2/rosbag2/issues/1552>`_)
* Improve performance in SqliteStorage::get_bagfile_size() (`#1516 <https://github.com/ros2/rosbag2/issues/1516>`_)
* Update rosbag2_storage_sqlite3 to C++17. (`#1501 <https://github.com/ros2/rosbag2/issues/1501>`_)
* Use enum values for offered_qos_profiles in code and string names in serialized metadata (`#1476 <https://github.com/ros2/rosbag2/issues/1476>`_)
* Stop inheriting from std::iterator. (`#1424 <https://github.com/ros2/rosbag2/issues/1424>`_)
* Contributors: Chris Lalancette, Michael Orlov, Patrick Roncagliolo, Roman Sokolkov

0.24.0 (2023-07-11)
-------------------
* Implement storing and loading ROS_DISTRO from metadata.yaml and mcap files (`#1241 <https://github.com/ros2/rosbag2/issues/1241>`_)
* Store metadata in db3 file (`#1294 <https://github.com/ros2/rosbag2/issues/1294>`_)
* Contributors: Emerson Knapp, Michael Orlov

0.23.0 (2023-04-28)
-------------------

0.22.0 (2023-04-18)
-------------------
* Add type_hash in MessageDefinition struct (`#1296 <https://github.com/ros2/rosbag2/issues/1296>`_)
* Store message definitions in SQLite3 storage plugin (`#1293 <https://github.com/ros2/rosbag2/issues/1293>`_)
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`_)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`_)
* Contributors: Michael Orlov, james-rms

0.21.0 (2023-04-12)
-------------------
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`_)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov, james-rms

0.20.0 (2023-02-14)
-------------------
* CLI: Get storage-specific values from plugin (`#1209 <https://github.com/ros2/rosbag2/issues/1209>`_)
* Contributors: Emerson Knapp

0.19.0 (2023-01-13)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_)
* Remove sqlite3-specific info from main README, make it more storage agnostic and point to plugin-specific README (`#1193 <https://github.com/ros2/rosbag2/issues/1193>`_)
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`_)
* Add `update_metadata(BagMetadata)` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`_)
* Store db schema version and ROS_DISTRO name in db3 files (`#1156 <https://github.com/ros2/rosbag2/issues/1156>`_)
* Contributors: Emerson Knapp, Michael Orlov, james-rms

0.18.0 (2022-11-15)
-------------------
* ros2bag: move storage preset validation to sqlite3 plugin (`#1135 <https://github.com/ros2/rosbag2/issues/1135>`_)
* Move sqlite3 storage implementation to rosbag2_storage_sqlite3 package (`#1113 <https://github.com/ros2/rosbag2/issues/1113>`_)
* Contributors: Emerson Knapp, james-rms

0.17.0 (2022-07-30)
-------------------
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`_)
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`_)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)
* Contributors: Esteve Fernandez

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
* Emit a warning rather than crash when a message is too big for sqlite (`#919 <https://github.com/ros2/rosbag2/issues/919>`_)
* Contributors: William Woodall

0.12.0 (2021-12-17)
-------------------
* Enable YAML encoding/decoding for RecordOptions and StorageOptions (`#916 <https://github.com/ros2/rosbag2/issues/916>`_)
* Contributors: Emerson Knapp

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Contributors: Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* added seek interface (`#836 <https://github.com/ros2/rosbag2/issues/836>`_)
* Contributors: sonia

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Reindexer core (`#641 <https://github.com/ros2/rosbag2/issues/641>`_)
  Add a new C++ Reindexer class for reconstructing metadata from bags that are missing it.
* Contributors: Emerson Knapp, jhdcs

0.7.0 (2021-03-18)
------------------

0.6.0 (2021-02-01)
------------------
* Fix build issues when rosbag2_storage is binary installed (`#585 <https://github.com/ros2/rosbag2/issues/585>`_)
* Mutex protection for db writing and stl collections in writer & storage (`#603 <https://github.com/ros2/rosbag2/issues/603>`_)
* SQLite storage optimized by default (`#568 <https://github.com/ros2/rosbag2/issues/568>`_)
* Contributors: Adam Dąbrowski, P. J. Reed

0.5.0 (2020-12-02)
------------------

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

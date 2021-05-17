^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.0 (2021-05-17)
------------------
* Naive clock jump implementation - allows for clock reuse and simplified Player setup (`#754 <https://github.com/ros2/rosbag2/issues/754>`_)
* Rename Reader/Writer 'reset' to 'close' (`#760 <https://github.com/ros2/rosbag2/issues/760>`_)
* Expose pause/resume related services on the Player (`#729 <https://github.com/ros2/rosbag2/issues/729>`_)
* player owns the reader (`#725 <https://github.com/ros2/rosbag2/issues/725>`_)
* Contributors: Emerson Knapp, Karsten Knese

0.8.0 (2021-04-19)
------------------
* Add set_rate to PlayerClock (`#727 <https://github.com/ros2/rosbag2/issues/727>`_)
* Enforce non-null now_fn in TimeControllerClock (`#731 <https://github.com/ros2/rosbag2/issues/731>`_)
* Fix pause snapshot behavior and add regression test (`#730 <https://github.com/ros2/rosbag2/issues/730>`_)
* Pause/resume PlayerClock (`#704 <https://github.com/ros2/rosbag2/issues/704>`_)
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* Enable thread safety analysis for rosbag2_cpp and add annotations in TimeControllerClock (`#710 <https://github.com/ros2/rosbag2/issues/710>`_)
* PlayerClock initial implementation - Player functionally unchanged (`#689 <https://github.com/ros2/rosbag2/issues/689>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Reindexer core (`#641 <https://github.com/ros2/rosbag2/issues/641>`_)
  Add a new C++ Reindexer class for reconstructing metadata from bags that are missing it.
* use rclcpp serialized messages to write data (`#457 <https://github.com/ros2/rosbag2/issues/457>`_)
* Contributors: Emerson Knapp, Karsten Knese, jhdcs

0.7.0 (2021-03-18)
------------------
* alternative write api (`#676 <https://github.com/ros2/rosbag2/issues/676>`_)
* RMW-implementation-searcher converter in rosbag2_cpp (`#670 <https://github.com/ros2/rosbag2/issues/670>`_)
* CLI query rosbag2_py for available storage implementations (`#659 <https://github.com/ros2/rosbag2/issues/659>`_)
* Fix --topics flag for ros2 bag play being ignored for all bags after the first one. (`#619 <https://github.com/ros2/rosbag2/issues/619>`_)
* Fix a crash in test_message_cache. (`#635 <https://github.com/ros2/rosbag2/issues/635>`_)
* Contributors: Alexander, Chris Lalancette, Emerson Knapp, Karsten Knese

0.6.0 (2021-02-01)
------------------
* Fix build issues when rosbag2_storage is binary installed (`#585 <https://github.com/ros2/rosbag2/issues/585>`_)
* Deduplicate SequentialCompressionReader business logic, add fallback to find bagfiles in incorrectly-written metadata (`#612 <https://github.com/ros2/rosbag2/issues/612>`_)
* include what you use (`#600 <https://github.com/ros2/rosbag2/issues/600>`_)
* Only dereference the data pointer if it is valid. (`#581 <https://github.com/ros2/rosbag2/issues/581>`_)
* Contributors: Chris Lalancette, Emerson Knapp, Ivan Santiago Paunovic, P. J. Reed

0.5.0 (2020-12-02)
------------------
* Add back rosbag2_cpp::StorageOptions as deprecated (`#563 <https://github.com/ros2/rosbag2/issues/563>`_)
* Sqlite storage double buffering (`#546 <https://github.com/ros2/rosbag2/issues/546>`_)
* Contributors: Adam Dąbrowski, Jacob Perron

0.4.0 (2020-11-19)
------------------
* correct master build (`#552 <https://github.com/ros2/rosbag2/issues/552>`_)
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Mutex around writer access in recorder (`#491 <https://github.com/ros2/rosbag2/issues/491>`_)
* if cache data exists, it needs to flush the data into the storage before shutdown (`#541 <https://github.com/ros2/rosbag2/issues/541>`_)
* Change default cache size for sequential_writer to a non zero value (`#533 <https://github.com/ros2/rosbag2/issues/533>`_)
* SequentialWriter to cache by message size instead of message count (`#530 <https://github.com/ros2/rosbag2/issues/530>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Remove some code duplication between SequentialWriter and SequentialCompressionWriter (`#527 <https://github.com/ros2/rosbag2/issues/527>`_)
* disable sanitizer by default (`#517 <https://github.com/ros2/rosbag2/issues/517>`_)
* Fix typo in error message (`#475 <https://github.com/ros2/rosbag2/issues/475>`_)
* introduce defaults for the C++ API (`#452 <https://github.com/ros2/rosbag2/issues/452>`_)
* Adding db directory creation to rosbag2_cpp (`#450 <https://github.com/ros2/rosbag2/issues/450>`_)
* comment out unused variable (`#460 <https://github.com/ros2/rosbag2/issues/460>`_)
* minimal c++ API test (`#451 <https://github.com/ros2/rosbag2/issues/451>`_)
* Add split by time to recording (`#409 <https://github.com/ros2/rosbag2/issues/409>`_)
* Contributors: Dirk Thomas, Jacob Perron, Jaison Titus, Karsten Knese, Marwan Taher, Michael Jeronimo, Patrick Spieler, jhdcs, Tomoya Fujita

0.3.2 (2020-06-03)
------------------
* Add user provided split size to error (`#430 <https://github.com/ros2/rosbag2/issues/430>`_)
  * Add user provided split size to error
  Signed-off-by: Anas Abou Allaban <aabouallaban@pm.me>
* Make split size error clearer (`#428 <https://github.com/ros2/rosbag2/issues/428>`_)
  Signed-off-by: Anas Abou Allaban <aabouallaban@pm.me>
* Contributors: Anas Abou Allaban

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Fix playback of compressed bagfiles (`#417 <https://github.com/ros2/rosbag2/issues/417>`_)
* Export targets (`#403 <https://github.com/ros2/rosbag2/issues/403>`_)
* Contributors: Emerson Knapp, Karsten Knese

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------
* Correct usage of rcpputils::SharedLibrary loading. (`#400 <https://github.com/ros2/rosbag2/issues/400>`_)
* Contributors: Karsten Knese

0.2.5 (2020-04-30)
------------------
* Don't fail build if lsan isn't available (`#397 <https://github.com/ros2/rosbag2/issues/397>`_)
* Expose BaseReaderInterface's BagMetadata  (`#377 <https://github.com/ros2/rosbag2/issues/377>`_)
* Expose topic filter to command line (addresses `#342 <https://github.com/ros2/rosbag2/issues/342>`_) (`#363 <https://github.com/ros2/rosbag2/issues/363>`_)
* Deduplicate code in SequentialCompressionReader (`#372 <https://github.com/ros2/rosbag2/issues/372>`_)
* rename rosidl_generator_c namespace to rosidl_runtime_c (`#368 <https://github.com/ros2/rosbag2/issues/368>`_)
* rename rosidl_generator_cpp namespace to rosidl_runtime_cpp (`#366 <https://github.com/ros2/rosbag2/issues/366>`_)
* added rosidl_runtime c and cpp depencencies (`#310 <https://github.com/ros2/rosbag2/issues/310>`_)
* Replace poco dependency by rcutils (`#322 <https://github.com/ros2/rosbag2/issues/322>`_)
* resolve relative file paths (`#345 <https://github.com/ros2/rosbag2/issues/345>`_)
* Add filter for reading selective topics (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* default max bag size to 0 (`#344 <https://github.com/ros2/rosbag2/issues/344>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* Add QoS to metadata (re-do `#330 <https://github.com/ros2/rosbag2/issues/330>`_) (`#335 <https://github.com/ros2/rosbag2/issues/335>`_)
* Revert "Add QoS profiles field to metadata struct and provide serialization utilities (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)" (`#334 <https://github.com/ros2/rosbag2/issues/334>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* Replace rcutils_get_file_size with rcpputils::fs::file_size (`#291 <https://github.com/ros2/rosbag2/issues/291>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Fix ros2 bag play on split bags (`#268 <https://github.com/ros2/rosbag2/issues/268>`_)
* [compression] Add SequentialCompressionWriter (`#260 <https://github.com/ros2/rosbag2/issues/260>`_)
* Add unit test for SequentialReader when metadata file does not exist (`#254 <https://github.com/ros2/rosbag2/issues/254>`_)
* Move compression artifacts from rosbag2_cpp to rosbag2_compression (`#257 <https://github.com/ros2/rosbag2/issues/257>`_)
* Fix uncrustify warnings (`#256 <https://github.com/ros2/rosbag2/issues/256>`_)
* remove rosbag2 filesystem helper (`#249 <https://github.com/ros2/rosbag2/issues/249>`_)
* [Compression - 8] Enable reader to read from compressed files/messages (`#246 <https://github.com/ros2/rosbag2/issues/246>`_)
* Make rosbag2 a metapackage (`#241 <https://github.com/ros2/rosbag2/issues/241>`_)
* Contributors: Alejandro Hernández Cordero, Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Scott K Logan, Sriram Raghunathan, Zachary Michaels

0.2.4 (2019-11-18)
------------------
* Load metadata from storage if no yaml file is found. (`#210 <https://github.com/ros2/rosbag2/issues/210>`_)
* Contributors: Karsten Knese

0.2.3 (2019-11-18)
------------------
* Enhance rosbag reader capabilities to read split bag files. (`#206 <https://github.com/ros2/rosbag2/issues/206>`_)
* Modular Reader/Writer API. (`#205 <https://github.com/ros2/rosbag2/issues/205>`_)
* Enhance rosbag writer capabilities to split bag files. (`#185 <https://github.com/ros2/rosbag2/issues/185>`_)
* Contributors: Karsten Knese, Zachary Michaels

0.2.2 (2019-11-13)
------------------
* (API) Generate bagfile metadata in Writer (`#184 <https://github.com/ros2/rosbag2/issues/184>`_)
* Contributors: Zachary Michaels

0.2.1 (2019-10-23)
------------------
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

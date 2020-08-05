^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.3.4 (2020-08-05)
------------------

0.3.3 (2020-06-23)
------------------

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

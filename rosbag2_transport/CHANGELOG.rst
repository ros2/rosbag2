^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2021-03-18)
------------------
* Add support for rmw_connextdds (`#671 <https://github.com/ros2/rosbag2/issues/671>`_)
* Use rosbag2_py for ros2 bag info (`#673 <https://github.com/ros2/rosbag2/issues/673>`_)
* Contributors: Andrea Sorbini, Karsten Knese

0.6.0 (2021-02-01)
------------------
* Fix build issues when rosbag2_storage is binary installed (`#585 <https://github.com/ros2/rosbag2/issues/585>`_)
* Regex and exclude fix for rosbag recorder (`#620 <https://github.com/ros2/rosbag2/issues/620>`_)
* Recorder --regex and --exclude options (`#604 <https://github.com/ros2/rosbag2/issues/604>`_)
* SQLite storage optimized by default (`#568 <https://github.com/ros2/rosbag2/issues/568>`_)
* Fixed playing if unknown message types exist (`#592 <https://github.com/ros2/rosbag2/issues/592>`_)
* Compress bag files in separate threads (`#506 <https://github.com/ros2/rosbag2/issues/506>`_)
* Stabilize test_record by reducing copies of executors and messages (`#576 <https://github.com/ros2/rosbag2/issues/576>`_)
* Contributors: Adam Dąbrowski, Chen Lihui, Emerson Knapp, P. J. Reed, Piotr Jaroszek

0.5.0 (2020-12-02)
------------------

0.4.0 (2020-11-19)
------------------
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* resolve memory leak for serialized message (`#502 <https://github.com/ros2/rosbag2/issues/502>`_)
* Use shared logic for importing the rosbag2_transport_py library in Python (`#482 <https://github.com/ros2/rosbag2/issues/482>`_)
* fix missing target dependencies (`#479 <https://github.com/ros2/rosbag2/issues/479>`_)
* reenable cppcheck for rosbag2_transport (`#461 <https://github.com/ros2/rosbag2/issues/461>`_)
* More reliable topic remapping test (`#456 <https://github.com/ros2/rosbag2/issues/456>`_)
* Add split by time to recording (`#409 <https://github.com/ros2/rosbag2/issues/409>`_)
* export shared_queues_vendor (`#434 <https://github.com/ros2/rosbag2/issues/434>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Karsten Knese, Michael Jeronimo, jhdcs

0.3.2 (2020-06-03)
------------------

0.3.1 (2020-06-01)
------------------
* Find rosbag2_cpp (tinyxml2) before rcl (`#423 <https://github.com/ros2/rosbag2/issues/423>`_)
* Shared publisher handle (`#420 <https://github.com/ros2/rosbag2/issues/420>`_)
* Contributors: Chris Lalancette, Karsten Knese

0.3.0 (2020-05-26)
------------------

0.2.8 (2020-05-18)
------------------
* Explicitly add DLL directories for Windows before importing (`#411 <https://github.com/ros2/rosbag2/issues/411>`_)
* Contributors: Jacob Perron

0.2.7 (2020-05-12)
------------------
* Remove MANUAL_BY_NODE liveliness usage (`#406 <https://github.com/ros2/rosbag2/issues/406>`_)
* Contributors: Ivan Santiago Paunovic

0.2.6 (2020-05-07)
------------------
* Correct usage of rcpputils::SharedLibrary loading. (`#400 <https://github.com/ros2/rosbag2/issues/400>`_)
* Contributors: Karsten Knese

0.2.5 (2020-04-30)
------------------
* add topic remapping option to rosbag2 play (`#388 <https://github.com/ros2/rosbag2/issues/388>`_)
* add missing test dependency (`#392 <https://github.com/ros2/rosbag2/issues/392>`_)
* use serialized message (`#386 <https://github.com/ros2/rosbag2/issues/386>`_)
* Adaptive playback qos based on recorded metadata (`#364 <https://github.com/ros2/rosbag2/issues/364>`_)
* Add loop option to rosbag play (`#361 <https://github.com/ros2/rosbag2/issues/361>`_)
* Move qos utilities to their own compilation unit (`#379 <https://github.com/ros2/rosbag2/issues/379>`_)
* Expose BaseReaderInterface's BagMetadata  (`#377 <https://github.com/ros2/rosbag2/issues/377>`_)
* Expose topic filter to command line (addresses `#342 <https://github.com/ros2/rosbag2/issues/342>`_) (`#363 <https://github.com/ros2/rosbag2/issues/363>`_)
* Fix Action CI tests to pass reliably (`#376 <https://github.com/ros2/rosbag2/issues/376>`_)
* Update GenericSubscription's handle_message signature (`#373 <https://github.com/ros2/rosbag2/issues/373>`_)
* Bridge CLI with transport (`#370 <https://github.com/ros2/rosbag2/issues/370>`_)
* Override QoS Profiles in CLI - Playback (`#356 <https://github.com/ros2/rosbag2/issues/356>`_)
* QoS Profile Overrides - Player (`#353 <https://github.com/ros2/rosbag2/issues/353>`_)
* Fix rosbag2_tests resource files and play_end_to_end test (`#362 <https://github.com/ros2/rosbag2/issues/362>`_)
* use ament_export_targets() (`#360 <https://github.com/ros2/rosbag2/issues/360>`_)
* Intelligently subscribe to topics according to their QoS profiles (`#355 <https://github.com/ros2/rosbag2/issues/355>`_)
* Add QoS Profile override to CLI (`#347 <https://github.com/ros2/rosbag2/issues/347>`_)
* Override Subscriber QoS - Record (`#346 <https://github.com/ros2/rosbag2/issues/346>`_)
* Replace poco dependency by rcutils (`#322 <https://github.com/ros2/rosbag2/issues/322>`_)
* Test all RMW implementations for rosbag2_transport (`#349 <https://github.com/ros2/rosbag2/issues/349>`_)
* Add filter for reading selective topics (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* Disable adaptive qos subscription for now  (`#348 <https://github.com/ros2/rosbag2/issues/348>`_)
* Subscribe to topics using the common offered QoS (`#343 <https://github.com/ros2/rosbag2/issues/343>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* Allow GenericPublisher / GenericSubscription to take a QoS profile (`#337 <https://github.com/ros2/rosbag2/issues/337>`_)
* Query offered QoS profiles for a topic and store in metadata (`#333 <https://github.com/ros2/rosbag2/issues/333>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* include hidden topics (`#332 <https://github.com/ros2/rosbag2/issues/332>`_)
* Add playback rate command line arg (`#304 <https://github.com/ros2/rosbag2/issues/304>`_)
* Removed rosidl_generator_cpp in rosbag2_transport because it's not used (`#321 <https://github.com/ros2/rosbag2/issues/321>`_)
* Fix race condition in transport recorder (`#303 <https://github.com/ros2/rosbag2/issues/303>`_)
* [compression] Enable compression through ros2bag cli (`#263 <https://github.com/ros2/rosbag2/issues/263>`_)
* code style only: wrap after open parenthesis if not in one line (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Make rosbag2 a metapackage (`#241 <https://github.com/ros2/rosbag2/issues/241>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Alejandro Hernández Cordero, Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Sriram Raghunathan, Zachary Michaels, carlossvg, ketatam

0.2.4 (2019-11-18)
------------------

0.2.3 (2019-11-18)
------------------
* Add CLI option to expose bagfile splitting. (`#203 <https://github.com/ros2/rosbag2/issues/203>`_)
* Delay subscriber asynchronous creation for opensplice in test_rosbag2_node. (`#196 <https://github.com/ros2/rosbag2/issues/196>`_)
* Modular Reader/Writer API. (`#205 <https://github.com/ros2/rosbag2/issues/205>`_)
* Contributors: Brian Marchi, Karsten Knese, Prajakta Gokhale

0.2.2 (2019-11-13)
------------------
* (API) Generate bagfile metadata in Writer (`#184 <https://github.com/ros2/rosbag2/issues/184>`_)
* Contributors: Zachary Michaels

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

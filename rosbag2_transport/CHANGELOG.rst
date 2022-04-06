^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Contributors: Audrow Nash, Barry Xu

0.15.0 (2022-04-05)
-------------------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Contributors: Barry Xu

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Add burst-mode to Player (`#977 <https://github.com/ros2/rosbag2/issues/977>`_)
* Install headers to include/${PROJECT_NAME} (`#958 <https://github.com/ros2/rosbag2/issues/958>`_)
* Make sure published messages are acknowledged for play mode (`#951 <https://github.com/ros2/rosbag2/issues/951>`_)
* Contributors: Barry Xu, Geoffrey Biggs, Shane Loretz

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------
* Changes for uncrustify 0.72 (`#937 <https://github.com/ros2/rosbag2/issues/937>`_)
* TopicFilter use regex_search instead of regex_match (`#932 <https://github.com/ros2/rosbag2/issues/932>`_)
* Add start-offset play option (`#931 <https://github.com/ros2/rosbag2/issues/931>`_)
* Add parentheses suggested by Clang on OSX to fix build warning (`#930 <https://github.com/ros2/rosbag2/issues/930>`_)
* Bag rewriter (C++) (`#920 <https://github.com/ros2/rosbag2/issues/920>`_)
* Add "ignore leaf topics" option to recorder (`#925 <https://github.com/ros2/rosbag2/issues/925>`_)
* Rewrite TopicFilter for single-call reusability (`#924 <https://github.com/ros2/rosbag2/issues/924>`_)
* Add a ReaderWriterFactory utility to share logic for reuse (`#923 <https://github.com/ros2/rosbag2/issues/923>`_)
* Add pause/resume options to the bag recorder (`#905 <https://github.com/ros2/rosbag2/issues/905>`_)
* Add logging macros for rosbag2_transport (`#917 <https://github.com/ros2/rosbag2/issues/917>`_)
* Enable YAML encoding/decoding for RecordOptions and StorageOptions (`#916 <https://github.com/ros2/rosbag2/issues/916>`_)
* Expose the QoS object wrapper (`#910 <https://github.com/ros2/rosbag2/issues/910>`_)
* Contributors: Abrar Rahman Protyasha, Chris Lalancette, Emerson Knapp, Geoffrey Biggs, Ivan Santiago Paunovic

0.11.0 (2021-11-08)
-------------------
* Add --start-paused option to `ros2 bag play` (`#904 <https://github.com/ros2/rosbag2/issues/904>`_)
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Add a Seek service (`#874 <https://github.com/ros2/rosbag2/issues/874>`_)
* Add simple keyboard control for playback rate (`#893 <https://github.com/ros2/rosbag2/issues/893>`_)
* Contributors: Emerson Knapp, Ivan Santiago Paunovic, Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Fix a bug on invalid pointer address when using "MESSAGE" compressio… (`#866 <https://github.com/ros2/rosbag2/issues/866>`_)
* Fix typo (`#880 <https://github.com/ros2/rosbag2/issues/880>`_)
* Use Reader's seek() method for seeking/jumping in Player (`#873 <https://github.com/ros2/rosbag2/issues/873>`_)
* keyboard controls for pause/resume toggle and play-next: (`#847 <https://github.com/ros2/rosbag2/issues/847>`_)
* Implement snapshot mechanism and corresponding ROS Service (`#850 <https://github.com/ros2/rosbag2/issues/850>`_)
* Circular Message Cache implementation for snapshot feature (`#844 <https://github.com/ros2/rosbag2/issues/844>`_)
* Add jump/seek API for Player class (`#826 <https://github.com/ros2/rosbag2/issues/826>`_)
* Restructure test_play_timing to one test per case, to see which times out (`#863 <https://github.com/ros2/rosbag2/issues/863>`_)
* Fix discovery silently stops after unknown msg type is found. (`#848 <https://github.com/ros2/rosbag2/issues/848>`_)
* Fixing deprecated subscriber callback warnings (`#852 <https://github.com/ros2/rosbag2/issues/852>`_)
* Bugfix for race condition in Player::peek_next_message_from_queue() (`#849 <https://github.com/ros2/rosbag2/issues/849>`_)
* added seek interface (`#836 <https://github.com/ros2/rosbag2/issues/836>`_)
* Update `PlayOptions::delay` to `rclcpp::Duration` to get nanosecond resolution (`#843 <https://github.com/ros2/rosbag2/issues/843>`_)
* Move notification about ready for playback inside play_messages_from_queue() (`#832 <https://github.com/ros2/rosbag2/issues/832>`_)
* Add wait for player to be ready for playback in Player::play_next() method (`#814 <https://github.com/ros2/rosbag2/issues/814>`_)
* Make sure the subscription exists before publishing messages (`#804 <https://github.com/ros2/rosbag2/issues/804>`_)
* Add delay option (`#789 <https://github.com/ros2/rosbag2/issues/789>`_)
* Copy recorder QoS profile to local variable so that temporary value isn't cleared (`#803 <https://github.com/ros2/rosbag2/issues/803>`_)
* test_play_services: fail gracefully on future error (`#798 <https://github.com/ros2/rosbag2/issues/798>`_)
* Recording with --all and --exclude fix (`#765 <https://github.com/ros2/rosbag2/issues/765>`_)
* Contributors: Abrar Rahman Protyasha, Barry Xu, Bastian Jäger, Cameron Miller, Emerson Knapp, Kosuke Takeuchi, Lei Liu, Louise Poubel, Michael Orlov, Piotr Jaroszek, sonia

0.9.0 (2021-05-17)
------------------
* Expose play_next service (`#767 <https://github.com/ros2/rosbag2/issues/767>`_)
* Add play_next() API to the player class (`#762 <https://github.com/ros2/rosbag2/issues/762>`_)
* Naive clock jump implementation - allows for clock reuse and simplified Player setup (`#754 <https://github.com/ros2/rosbag2/issues/754>`_)
* Rename Reader/Writer 'reset' to 'close' (`#760 <https://github.com/ros2/rosbag2/issues/760>`_)
* simply constructor for rosbag2_transport::Player (`#757 <https://github.com/ros2/rosbag2/issues/757>`_)
* Expose GetRate/SetRate services for playback (`#753 <https://github.com/ros2/rosbag2/issues/753>`_)
* Expose pause/resume related services on the Player (`#729 <https://github.com/ros2/rosbag2/issues/729>`_)
* remodel publication manager (`#749 <https://github.com/ros2/rosbag2/issues/749>`_)
* remove rosbag2_transport header (`#742 <https://github.com/ros2/rosbag2/issues/742>`_)
* use public recorder api in tests (`#741 <https://github.com/ros2/rosbag2/issues/741>`_)
* Use public player API in tests (`#740 <https://github.com/ros2/rosbag2/issues/740>`_)
* public recorder and player (`#739 <https://github.com/ros2/rosbag2/issues/739>`_)
* player owns the reader (`#725 <https://github.com/ros2/rosbag2/issues/725>`_)
* Contributors: Emerson Knapp, Karsten Knese, Michael Orlov

0.8.0 (2021-04-19)
------------------
* cleanup cmakelists (`#726 <https://github.com/ros2/rosbag2/issues/726>`_)
* turn recorder into a node (`#724 <https://github.com/ros2/rosbag2/issues/724>`_)
* turn player into a node (`#723 <https://github.com/ros2/rosbag2/issues/723>`_)
* Remove -Werror from builds, enable it in Action CI (`#722 <https://github.com/ros2/rosbag2/issues/722>`_)
* Split Rosbag2Transport into Player and Recorder classes - first pass to enable further progress (`#721 <https://github.com/ros2/rosbag2/issues/721>`_)
* /clock publisher in Player (`#695 <https://github.com/ros2/rosbag2/issues/695>`_)
* use rclcpp logging macros (`#715 <https://github.com/ros2/rosbag2/issues/715>`_)
* use rclcpp::Node for generic pub/sub (`#714 <https://github.com/ros2/rosbag2/issues/714>`_)
* PlayerClock initial implementation - Player functionally unchanged (`#689 <https://github.com/ros2/rosbag2/issues/689>`_)
* Fix bad_function_call by replacing rclcpp::spin_some with SingleThreadedExecutor (`#705 <https://github.com/ros2/rosbag2/issues/705>`_)
* rosbag2_py pybind wrapper for "record" - remove rosbag2_transport_py (`#702 <https://github.com/ros2/rosbag2/issues/702>`_)
* Add rosbag2_py::Player::play to replace rosbag2_transport_python version (`#693 <https://github.com/ros2/rosbag2/issues/693>`_)
* Fix and clarify logic in test_play filter test (`#690 <https://github.com/ros2/rosbag2/issues/690>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Add QoS decoding translation for infinite durations to RMW_DURATION_INFINITE (`#684 <https://github.com/ros2/rosbag2/issues/684>`_)
* Contributors: Emerson Knapp, Karsten Knese

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

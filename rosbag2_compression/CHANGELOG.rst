^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_compression
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.5 (2023-11-18)
-------------------

0.22.4 (2023-10-24)
-------------------

0.22.3 (2023-09-08)
-------------------

0.22.2 (2023-07-14)
-------------------

0.22.1 (2023-05-18)
-------------------
* Add in a missing cstdint include. (`#1321 <https://github.com/ros2/rosbag2/issues/1321>`_) (`#1322 <https://github.com/ros2/rosbag2/issues/1322>`_)
* Fix warning from ClassLoader in sequential compression reader and writer (`#1299 <https://github.com/ros2/rosbag2/issues/1299>`_) (`#1316 <https://github.com/ros2/rosbag2/issues/1316>`_)
* Contributors: mergify[bot]

0.22.0 (2023-04-18)
-------------------
* Add message definition read API (`#1292 <https://github.com/ros2/rosbag2/issues/1292>`_)
* rosbag2_storage: add type description hash to topic metadata (`#1272 <https://github.com/ros2/rosbag2/issues/1272>`_)
* Contributors: james-rms

0.21.0 (2023-04-12)
-------------------
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`_)
* Update rosbag2 to C++17. (`#1238 <https://github.com/ros2/rosbag2/issues/1238>`_)
* Use target_link_libraries instead of ament_target_dependencies (`#1202 <https://github.com/ros2/rosbag2/issues/1202>`_)
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Michael Orlov, james-rms

0.20.0 (2023-02-14)
-------------------

0.19.0 (2023-01-13)
-------------------
* set_read_order: return success (`#1177 <https://github.com/ros2/rosbag2/issues/1177>`_)
* Add `update_metadata(BagMetadata)` API for storage plugin interface (`#1149 <https://github.com/ros2/rosbag2/issues/1149>`_)
* Contributors: Michael Orlov, james-rms

0.18.0 (2022-11-15)
-------------------
* Reverse read order API and sqlite storage implementation (`#1083 <https://github.com/ros2/rosbag2/issues/1083>`_)
* Add option to prevent message loss while converting (`#1058 <https://github.com/ros2/rosbag2/issues/1058>`_)
* set default metadata of compressed message (in case compressor does not set it) (`#1060 <https://github.com/ros2/rosbag2/issues/1060>`_)
* Contributors: DensoADAS, Emerson Knapp

0.17.0 (2022-07-30)
-------------------
* Speed optimization: Preparing copyless publish/subscribing by using const message for writing (`#1010 <https://github.com/ros2/rosbag2/issues/1010>`_)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`_)
* Contributors: DensoADAS, Hunter L. Allen, Joshua Hampp, Michael Orlov, Tony Peng

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
* Remove unnecessary public definition. (`#950 <https://github.com/ros2/rosbag2/issues/950>`_)
* Contributors: Chris Lalancette, Shane Loretz

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------
* Changes for uncrustify 0.72 (`#937 <https://github.com/ros2/rosbag2/issues/937>`_)
* Bugfix for broken bag split when using cache (`#936 <https://github.com/ros2/rosbag2/issues/936>`_)
* Contributors: Chris Lalancette, Michael Orlov

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Don't preprocess a storage file more than once (`#895 <https://github.com/ros2/rosbag2/issues/895>`_)
* Contributors: Michel Hidalgo, sonia

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* added seek interface (`#836 <https://github.com/ros2/rosbag2/issues/836>`_)
* Refactor plugin query mechanism and standardize trait management (`#833 <https://github.com/ros2/rosbag2/issues/833>`_)
* fix sequential reader rollover-to-next-file strategy: (`#839 <https://github.com/ros2/rosbag2/issues/839>`_)
* Load compression and serialization choices via plugin query (`#827 <https://github.com/ros2/rosbag2/issues/827>`_)
* Contributors: Cameron Miller, sonia

0.9.0 (2021-05-17)
------------------
* Rename Reader/Writer 'reset' to 'close' (`#760 <https://github.com/ros2/rosbag2/issues/760>`_)
* Contributors: Emerson Knapp

0.8.0 (2021-04-19)
------------------
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Reindexer core (`#641 <https://github.com/ros2/rosbag2/issues/641>`_)
* Contributors: Emerson Knapp, jhdcs

0.7.0 (2021-03-18)
------------------
* CLI query rosbag2_py for available storage implementations (`#659 <https://github.com/ros2/rosbag2/issues/659>`_)
* Move zstd compressor to its own package (`#636 <https://github.com/ros2/rosbag2/issues/636>`_)
* Remove rosbag2_compression test dependencies on zstd implementation in prep for moving it into a separate package (`#637 <https://github.com/ros2/rosbag2/issues/637>`_)
* Contributors: Emerson Knapp

0.6.0 (2021-02-01)
------------------
* Make compressor implementations into a plugin via pluginlib (`#624 <https://github.com/ros2/rosbag2/issues/624>`_)
* Use ZSTD's streaming interface for [de]compressing files (`#543 <https://github.com/ros2/rosbag2/issues/543>`_)
* Fix build issues when rosbag2_storage is binary installed (`#585 <https://github.com/ros2/rosbag2/issues/585>`_)
* Fix relative metadata paths in SequentialCompressionWriter (`#613 <https://github.com/ros2/rosbag2/issues/613>`_)
* Fix deadlock race condition on compression shutdown (`#616 <https://github.com/ros2/rosbag2/issues/616>`_)
* Deduplicate SequentialCompressionReader business logic, add fallback to find bagfiles in incorrectly-written metadata (`#612 <https://github.com/ros2/rosbag2/issues/612>`_)
* Compress bag files in separate threads (`#506 <https://github.com/ros2/rosbag2/issues/506>`_)
* Contributors: Emerson Knapp, P. J. Reed

0.5.0 (2020-12-02)
------------------
* Sqlite storage double buffering (`#546 <https://github.com/ros2/rosbag2/issues/546>`_)
* Contributors: Adam Dąbrowski

0.4.0 (2020-11-19)
------------------
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* Do not expect empty StorageOptions URI to work in CompressionWriterTest (`#526 <https://github.com/ros2/rosbag2/issues/526>`_)
* Remove some code duplication between SequentialWriter and SequentialCompressionWriter (`#527 <https://github.com/ros2/rosbag2/issues/527>`_)
* Fix exception thrown given invalid arguments with compression enabled (`#488 <https://github.com/ros2/rosbag2/issues/488>`_)
* Adding db directory creation to rosbag2_cpp (`#450 <https://github.com/ros2/rosbag2/issues/450>`_)
* Consolidate ZSTD utility functions (`#459 <https://github.com/ros2/rosbag2/issues/459>`_)
* Add per-message ZSTD compression (`#418 <https://github.com/ros2/rosbag2/issues/418>`_)
* Contributors: Christophe Bedard, Devin Bonnie, Jaison Titus, Karsten Knese, Marwan Taher, Michael Jeronimo, P. J. Reed

0.3.2 (2020-06-03)
------------------
* Add user provided split size to error message (`#430 <https://github.com/ros2/rosbag2/issues/430>`_)
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
* Remove relative include paths in rosbag2_compression tests (`#405 <https://github.com/ros2/rosbag2/issues/405>`_)
* Contributors: Karsten Knese, Zachary Michaels

0.2.5 (2020-04-30)
------------------
* Don't fail build if lsan isn't available. (`#397 <https://github.com/ros2/rosbag2/issues/397>`_)
* Correctly set all test dependencies. (`#392 <https://github.com/ros2/rosbag2/issues/392>`_)
* Deduplicate code in SequentialCompressionReader. (`#372 <https://github.com/ros2/rosbag2/issues/372>`_)
* Add filter for reading selective topics. (`#302 <https://github.com/ros2/rosbag2/issues/302>`_)
* Add QoS profiles field to metadata struct and provide serialization utilities. (`#330 <https://github.com/ros2/rosbag2/issues/330>`_)
* Fix compression log logic. (`#320 <https://github.com/ros2/rosbag2/issues/320>`_)
* Fix throw in playback of split+compressed bagfiles. (`#294 <https://github.com/ros2/rosbag2/issues/294>`_)
* Refactor Compression Reader/Writers to use the CompressionFactory. (`#315 <https://github.com/ros2/rosbag2/issues/315>`_)
* Add compression factory implementation. (`#313 <https://github.com/ros2/rosbag2/issues/313>`_)
* Include stdexcept. (`#314 <https://github.com/ros2/rosbag2/issues/314>`_)
* Add compression factory stubs. (`#311 <https://github.com/ros2/rosbag2/issues/311>`_)
* Replace rcutils_get_file_size with rcpputils::fs::file_size. (`#291 <https://github.com/ros2/rosbag2/issues/291>`_)
* [compression] Enable compression through ros2bag cli. (`#263 <https://github.com/ros2/rosbag2/issues/263>`_)
* [compression] Close storage before compression. (`#284 <https://github.com/ros2/rosbag2/issues/284>`_)
* Improve logging in rosbag2_compression. (`#287 <https://github.com/ros2/rosbag2/issues/287>`_)
* Change validation functions to accept output type of ZSTD_getFrameContentSize. (`#285 <https://github.com/ros2/rosbag2/issues/285>`_)
* code style only: wrap after open parenthesis if not in one line. (`#280 <https://github.com/ros2/rosbag2/issues/280>`_)
* Add more assertions on rosbag2_compression. (`#279 <https://github.com/ros2/rosbag2/issues/279>`_)
* [compression] Add SequentialCompressionWriter. (`#260 <https://github.com/ros2/rosbag2/issues/260>`_)
* Add a SequentialCompressionReader. (`#258 <https://github.com/ros2/rosbag2/issues/258>`_)
* Move compression artifacts from rosbag2_cpp to rosbag2_compression. (`#257 <https://github.com/ros2/rosbag2/issues/257>`_)
* remove rosbag2 filesystem helper. (`#249 <https://github.com/ros2/rosbag2/issues/249>`_)
* [Compression - 8] Enable reader to read from compressed files/messages. (`#246 <https://github.com/ros2/rosbag2/issues/246>`_)
* [compression] Follow ROS2 style conventions better and throw eagerly. (`#245 <https://github.com/ros2/rosbag2/issues/245>`_)
* [Compression] Use vector resize instead of reserve. (`#243 <https://github.com/ros2/rosbag2/issues/243>`_)
* [Compression - 6] Add Zstd file decompression implementation. (`#230 <https://github.com/ros2/rosbag2/issues/230>`_)
* Check output of fread/fwrite in compression. (`#237 <https://github.com/ros2/rosbag2/issues/237>`_)
* Fix compress uri. (`#234 <https://github.com/ros2/rosbag2/issues/234>`_)
* [Compression - 5] Add Zstd file compression. (`#220 <https://github.com/ros2/rosbag2/issues/220>`_)
* [Compression - 4] Add decompressor interface. (`#219 <https://github.com/ros2/rosbag2/issues/219>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Emerson Knapp, Karsten Knese, Mabel Zhang, Scott K Logan, Thomas Moulard, Zachary Michaels

0.2.4 (2019-11-18 17:51)
------------------------

0.2.3 (2019-11-18 13:55)
------------------------

0.2.2 (2019-11-13)
------------------

0.2.1 (2019-10-23)
------------------

0.2.0 (2019-09-26)
------------------

0.1.2 (2019-05-20)
------------------

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------

0.0.3 (2018-12-14)
------------------

0.0.2 (2018-12-12)
------------------

0.0.1 (2018-12-11)
------------------

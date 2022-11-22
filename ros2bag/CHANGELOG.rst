^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2bag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.18.0 (2022-11-15)
-------------------
* rosbag2_storage: expose default storage ID as method (`#1146 <https://github.com/ros2/rosbag2/issues/1146>`_)
* Fix for ros2 bag play exit with non-zero code on SIGINT (`#1126 <https://github.com/ros2/rosbag2/issues/1126>`_)
* ros2bag: move storage preset validation to sqlite3 plugin (`#1135 <https://github.com/ros2/rosbag2/issues/1135>`_)
* Add option to prevent message loss while converting (`#1058 <https://github.com/ros2/rosbag2/issues/1058>`_)
* Added support for excluding topics via regular expressions (`#1046 <https://github.com/ros2/rosbag2/issues/1046>`_)
* Readers/info can accept a single bag storage file, and detect its storage id automatically (`#1072 <https://github.com/ros2/rosbag2/issues/1072>`_)
* Add short -v option to ros2 bag list for verbose (`#1065 <https://github.com/ros2/rosbag2/issues/1065>`_)
* Contributors: DensoADAS, Emerson Knapp, Esteve Fernandez, Michael Orlov, james-rms

0.17.0 (2022-07-30)
-------------------
* Use a single variable for evaluating the filter regex (`#1053 <https://github.com/ros2/rosbag2/issues/1053>`_)
* Add additional mode of publishing sim time updates triggered by replayed messages (`#1050 <https://github.com/ros2/rosbag2/issues/1050>`_)
 * When this mode is active, /clock updates are triggered whenever messages are replayed rather
   than at a fixed rate. Optionally, a list of triggering topics can be set so that only a subset
   of replayed messages will trigger the /clock update. This mode is most useful when replaying
   applications which do some sanity checking or correlation of message data to system timestamps.
   If the application does not need the sim time to be updated at a consistent rate, this mode can
   substantially reduce the overhead of having sim time enabled in rosbag2.
* Renamed --topics-regex to --regex and -e in Player class to be consistent with Recorder (`#1045 <https://github.com/ros2/rosbag2/issues/1045>`_)
* Use first available writer in recording if default `sqlite3` not available. (`#1044 <https://github.com/ros2/rosbag2/issues/1044>`_)
* Add the ability to record any key/value pair in 'custom' field in metadata.yaml (`#1038 <https://github.com/ros2/rosbag2/issues/1038>`_)
* Added support for filtering topics via regular expressions on Playback (`#1034 <https://github.com/ros2/rosbag2/issues/1034>`_)
* Fix incorrect boundary check for `playback_duration` and `play_until_timestamp` (`#1032 <https://github.com/ros2/rosbag2/issues/1032>`_)
 * Add initialization for `metadata  starting time` in MockSequentialReader
 * Fixed one false positive and one flaky test in test_play_until
* Adds play until timestamp functionality (`#1005 <https://github.com/ros2/rosbag2/issues/1005>`_)
* Add CLI verb for burst mode of playback (`#980 <https://github.com/ros2/rosbag2/issues/980>`_)
* Add play-for specified number of seconds functionality (`#960 <https://github.com/ros2/rosbag2/issues/960>`_)
* Contributors: Agustin Alba Chicar, EsipovPA, Esteve Fernandez, Geoffrey Biggs, Hunter L.Allen,
  Michael Orlov, kylemarcey, Tony Peng

0.16.0 (2022-05-11)
-------------------
* Make unpublished topics unrecorded by default (`#968 <https://github.com/ros2/rosbag2/issues/968>`_)
* Contributors: Michael Orlov, Sean Kelly

0.15.1 (2022-04-06)
-------------------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Audrow Nash, Barry Xu, Jorge Perez, Tony Peng

0.15.0 (2022-04-05)
-------------------
* support to publish as loaned message (`#981 <https://github.com/ros2/rosbag2/issues/981>`_)
* Revert "Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)" (`#984 <https://github.com/ros2/rosbag2/issues/984>`_)
* Add the ability to record any key/value pair in the 'custom' field in metadata.yaml (`#976 <https://github.com/ros2/rosbag2/issues/976>`_)
* Contributors: Barry Xu, Jorge Perez, Tony Peng

0.14.1 (2022-03-29)
-------------------
* Bump version number to avoid conflict
* Contributors: Chris Lalancette

0.14.0 (2022-03-29)
-------------------
* Make sure published messages are acknowledged for play mode (`#951 <https://github.com/ros2/rosbag2/issues/951>`_)
* Contributors: Barry Xu

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------
* TopicFilter use regex_search instead of regex_match (`#932 <https://github.com/ros2/rosbag2/issues/932>`_)
* Add start-offset play option (`#931 <https://github.com/ros2/rosbag2/issues/931>`_)
* Expose bag_rewrite as `ros2 bag convert` (`#921 <https://github.com/ros2/rosbag2/issues/921>`_)
* Add "ignore leaf topics" option to recorder (`#925 <https://github.com/ros2/rosbag2/issues/925>`_)
* Auto-detect storage_id for Reader (if possible) (`#918 <https://github.com/ros2/rosbag2/issues/918>`_)
* Add pause/resume options to the bag recorder (`#905 <https://github.com/ros2/rosbag2/issues/905>`_)
* Contributors: Abrar Rahman Protyasha, Emerson Knapp, Ivan Santiago Paunovic

0.11.0 (2021-11-08)
-------------------
* Add --start-paused option to `ros2 bag play` (`#904 <https://github.com/ros2/rosbag2/issues/904>`_)
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Fix converter plugin choices for record (`#897 <https://github.com/ros2/rosbag2/issues/897>`_)
* Contributors: Emerson Knapp, Ivan Santiago Paunovic, Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------
* Add missing spaces to error message (`#875 <https://github.com/ros2/rosbag2/issues/875>`_)
* keyboard controls for pause/resume toggle and play-next: (`#847 <https://github.com/ros2/rosbag2/issues/847>`_)
* Add --snapshot-mode argument to the "record" verb (`#851 <https://github.com/ros2/rosbag2/issues/851>`_)
* Refactor plugin query mechanism and standardize trait management (`#833 <https://github.com/ros2/rosbag2/issues/833>`_)
* Update `PlayOptions::delay` to `rclcpp::Duration` to get nanosecond resolution (`#843 <https://github.com/ros2/rosbag2/issues/843>`_)
* Load compression and serialization choices via plugin query (`#827 <https://github.com/ros2/rosbag2/issues/827>`_)
* Add delay option (`#789 <https://github.com/ros2/rosbag2/issues/789>`_)
* Avoid passing exception KeyboardInterrupt to the upper layer (`#788 <https://github.com/ros2/rosbag2/issues/788>`_)
* Contributors: Barry Xu, Cameron Miller, Emerson Knapp, Jacob Perron, Kosuke Takeuchi, Sonia Jin

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------
* /clock publisher in Player (`#695 <https://github.com/ros2/rosbag2/issues/695>`_)
* Introducing Reindexer CLI (`#699 <https://github.com/ros2/rosbag2/issues/699>`_)
* rosbag2_py pybind wrapper for "record" - remove rosbag2_transport_py (`#702 <https://github.com/ros2/rosbag2/issues/702>`_)
* Add rosbag2_py::Player::play to replace rosbag2_transport_python version (`#693 <https://github.com/ros2/rosbag2/issues/693>`_)
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Contributors: Emerson Knapp, jhdcs

0.7.0 (2021-03-18)
------------------
* use rosbag2_py for ros2 bag info (`#673 <https://github.com/ros2/rosbag2/issues/673>`_)
* CLI query rosbag2_py for available storage implementations (`#659 <https://github.com/ros2/rosbag2/issues/659>`_)
* Contributors: Emerson Knapp, Karsten Knese

0.6.0 (2021-02-01)
------------------
* Recorder --regex and --exclude options (`#604 <https://github.com/ros2/rosbag2/issues/604>`_)
* Fix the tests on cyclonedds by translating qos duration values (`#606 <https://github.com/ros2/rosbag2/issues/606>`_)
* SQLite storage optimized by default (`#568 <https://github.com/ros2/rosbag2/issues/568>`_)
* Fix a bug on parsing wrong description in plugin xml file (`#578 <https://github.com/ros2/rosbag2/issues/578>`_)
* Compress bag files in separate threads (`#506 <https://github.com/ros2/rosbag2/issues/506>`_)
* Contributors: Adam Dąbrowski, Barry Xu, Emerson Knapp, P. J. Reed

0.5.0 (2020-12-02)
------------------
* Sqlite storage double buffering (`#546 <https://github.com/ros2/rosbag2/issues/546>`_)
  * Double buffers
  * Circular queue and FLUSH option as define
  * Minor naming and lexical fixes.
  * Removed FLUSH_BUFFERS define check.
  * Sqlite3 storage logging fixes.
  * Sqlite3 storage circular buffer with pre allocated memory.
  * Sqlite3 storage buffers moved to shared_ptrs.
  * Uncrustify
  * Moved double buffers to writer
  * Buffer layer reset in seq compression writer in rosbag2 cpp
  * Buffer layer for rosbag2 writer refactor
  * Changed buffers in BufferLayer to std vectors.
  * BufferLayer uncrustify
  * Removed non-applicable test for writer cache.
  * BufferLayer review fixes
  * Rosbag metadata msgs count fixed for BufferLayer
  * Condition variable for buffer layer sync.
  * Fixed buffer locks
  * Buffers in BufferLayer refactored, moved into new class
  * Buffer layer split bags fixed.
  * Storage options include fix in buffer layer header.
  * Mutex around swapping buffers in buffer layer.
  * Fixed cache 0 bug in buffer layer.
  * Minor buffer layer refactor.
  * Counting messages in writer refactored.
  * Changed default cache size to 100Mb and updated parameter description
  * Applied review remarks:
  - significant refactoring: separation of cache classes
  - applied suggested improvements
  - some renaming
  - reduce code duplication that would otherwise increase with cache refactor, between compression and plain writers
  * Applied review comments
  - cache consumer now takes a callback and is independent of storage
  - namespace changes, renaming, cleaning
  - counting and logging messages by topic
  * linter
  * Changes after review: fixing flushing, topic counts, and more
  * Fix for splitting - flushing state now correctly turns off
  * cache classes documentation
  * simplified signature
  * a couple of tests for cache
  * address review: explicit constructor and doxygen styling
  * Windows warnings fix
  * fixed type mismatch warning on Windows
  * added minor comment
  Co-authored-by: Piotr Jaroszek <piotr.jaroszek@robotec.ai>
* Contributors: Adam Dąbrowski

0.4.0 (2020-11-19)
------------------
* read yaml config file (`#497 <https://github.com/ros2/rosbag2/issues/497>`_)
* List all storage plugins in plugin xml file (`#554 <https://github.com/ros2/rosbag2/issues/554>`_)
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update deprecated qos policy value names (`#548 <https://github.com/ros2/rosbag2/issues/548>`_)
* Add record test for ros2bag (`#523 <https://github.com/ros2/rosbag2/issues/523>`_)
* Removed duplicated code in record (`#534 <https://github.com/ros2/rosbag2/issues/534>`_)
* Change default cache size for sequential_writer to a non zero value (`#533 <https://github.com/ros2/rosbag2/issues/533>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* [ros2bag test_record] Gets rid of time.sleep and move to using command.wait_for_output (`#525 <https://github.com/ros2/rosbag2/issues/525>`_)
* Add pytest.ini back to ros2bag. (`#492 <https://github.com/ros2/rosbag2/issues/492>`_)
* performance testing packages (`#442 <https://github.com/ros2/rosbag2/issues/442>`_)
* Validate QoS profile values are not negative. (`#483 <https://github.com/ros2/rosbag2/issues/483>`_)
* catch parent exception (`#472 <https://github.com/ros2/rosbag2/issues/472>`_)
* add wait for closed file handles on Windows (`#470 <https://github.com/ros2/rosbag2/issues/470>`_)
* introduce ros2 bag list <plugins> (`#468 <https://github.com/ros2/rosbag2/issues/468>`_)
* move wait_for_shutdown() call out of the context manager (`#466 <https://github.com/ros2/rosbag2/issues/466>`_)
* Adding db directory creation to rosbag2_cpp (`#450 <https://github.com/ros2/rosbag2/issues/450>`_)
* use a single temp dir for the test class (`#462 <https://github.com/ros2/rosbag2/issues/462>`_)
* Add per-message ZSTD compression (`#418 <https://github.com/ros2/rosbag2/issues/418>`_)
* Add split by time to recording (`#409 <https://github.com/ros2/rosbag2/issues/409>`_)
* Add pytest.ini so local tests don't display warning (`#446 <https://github.com/ros2/rosbag2/issues/446>`_)
* Contributors: Adam Dąbrowski, Barry Xu, Chris Lalancette, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Jaison Titus, Jesse Ikawa, Karsten Knese, Marwan Taher, Michael Jeronimo, P. J. Reed, jhdcs

0.3.2 (2020-06-03)
------------------
* Improve help message for CLI verbs (`#427 <https://github.com/ros2/rosbag2/issues/427>`_)
* Contributors: Jacob Perron

0.3.1 (2020-06-01)
------------------

0.3.0 (2020-05-26)
------------------
* Don't allow user to specify unimplemented compression mode 'message' (`#415 <https://github.com/ros2/rosbag2/issues/415>`_)
* Use consistent quotes in help messages (`#416 <https://github.com/ros2/rosbag2/issues/416>`_)
* Contributors: Dirk Thomas, Emerson Knapp

0.2.8 (2020-05-18)
------------------

0.2.7 (2020-05-12)
------------------

0.2.6 (2020-05-07)
------------------

0.2.5 (2020-04-30)
------------------
* add topic remapping option to rosbag2 play (`#388 <https://github.com/ros2/rosbag2/issues/388>`_)
* Add loop option to rosbag play (`#361 <https://github.com/ros2/rosbag2/issues/361>`_)
* Expose topic filter to command line (addresses `#342 <https://github.com/ros2/rosbag2/issues/342>`_) (`#363 <https://github.com/ros2/rosbag2/issues/363>`_)
* Override QoS Profiles in CLI - Playback (`#356 <https://github.com/ros2/rosbag2/issues/356>`_)
* Refactor utility functions in ros2bag (`#358 <https://github.com/ros2/rosbag2/issues/358>`_)
* Add QoS Profile override to CLI (`#347 <https://github.com/ros2/rosbag2/issues/347>`_)
* Transaction based sqlite3 inserts (`#225 <https://github.com/ros2/rosbag2/issues/225>`_)
* include hidden topics (`#332 <https://github.com/ros2/rosbag2/issues/332>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Add playback rate command line arg (`#304 <https://github.com/ros2/rosbag2/issues/304>`_)
* [compression] Enable compression through ros2bag cli (`#263 <https://github.com/ros2/rosbag2/issues/263>`_)
* switch to not deprecated API (`#261 <https://github.com/ros2/rosbag2/issues/261>`_)
* make ros tooling working group maintainer (`#211 <https://github.com/ros2/rosbag2/issues/211>`_)
* Contributors: Anas Abou Allaban, Dirk Thomas, Karsten Knese, Mabel Zhang, Sriram Raghunathan, Zachary Michaels, ketatam

0.2.4 (2019-11-18)
------------------

0.2.3 (2019-11-18)
------------------
* Add CLI option to expose option for bagfile splitting (`#203 <https://github.com/ros2/rosbag2/issues/203>`_)
* Contributors: Karsten Knese, Prajakta Gokhale

0.2.2 (2019-11-13)
------------------

0.2.1 (2019-10-23)
------------------
* Fix flake8 errors and add missing lint tests. (`#194 <https://github.com/ros2/rosbag2/issues/194>`_)
* Import rosbag2_transport Python module on demand. (`#190 <https://github.com/ros2/rosbag2/issues/190>`_)
* Contributors: Michel Hidalgo, Thomas Moulard

0.2.0 (2019-09-26)
------------------
* install resource marker file for package (`#167 <https://github.com/ros2/rosbag2/issues/167>`_)
* install package manifest (`#161 <https://github.com/ros2/rosbag2/issues/161>`_)
* Contributors: Dirk Thomas, Ruffin

0.1.2 (2019-05-20)
------------------
* remove disclaimer (`#122 <https://github.com/ros2/rosbag2/issues/122>`_)
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.1.1 (2019-05-09)
------------------

0.1.0 (2019-05-08)
------------------
* Fix issue with ros2bag record python frontend (`#100 <https://github.com/ros2/rosbag2/issues/100>`_)
* Consistent node naming across ros2cli tools (`#60 <https://github.com/ros2/rosbag2/issues/60>`_)
* Contributors: AAlon, Sagnik Basu

0.0.5 (2018-12-27)
------------------

0.0.4 (2018-12-19)
------------------
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
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
* Use directory as bagfile and add additonal record options (`#43 <https://github.com/ros2/rosbag2/issues/43>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
* initial command line interface (`#12 <https://github.com/ros2/rosbag2/issues/12>`_)
* (demo, sqlite3) First working rosbag2 implementation (`#6 <https://github.com/ros2/rosbag2/issues/6>`_)
* initial setup
* Contributors: Alessandro Bottero, Andreas Greimel, Karsten Knese, Martin Idel

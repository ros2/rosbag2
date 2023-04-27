^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.15.5 (2023-04-25)
-------------------
* Add Michael Orlov as maintainer in rosbag2 packages (`#1215 <https://github.com/ros2/rosbag2/issues/1215>`_) (`#1224 <https://github.com/ros2/rosbag2/issues/1224>`_)
* Contributors: mergify[bot]

0.15.4 (2023-01-10)
-------------------

0.15.3 (2022-11-07)
-------------------

0.15.2 (2022-05-11)
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

0.13.0 (2022-01-13)
-------------------

0.12.0 (2021-12-17)
-------------------

0.11.0 (2021-11-08)
-------------------
* Update package maintainers (`#899 <https://github.com/ros2/rosbag2/issues/899>`_)
* Contributors: Michel Hidalgo

0.10.1 (2021-10-22)
-------------------

0.10.0 (2021-10-19)
-------------------

0.9.0 (2021-05-17)
------------------

0.8.0 (2021-04-19)
------------------
* Explicitly add emersonknapp as maintainer (`#692 <https://github.com/ros2/rosbag2/issues/692>`_)
* Contributors: Emerson Knapp

0.7.0 (2021-03-18)
------------------
* RMW-implementation-searcher converter in rosbag2_cpp (`#670 <https://github.com/ros2/rosbag2/issues/670>`_)
* Move zstd compressor to its own package (`#636 <https://github.com/ros2/rosbag2/issues/636>`_)
* Contributors: Emerson Knapp

0.6.0 (2021-02-01)
------------------

0.5.0 (2020-12-02)
------------------

0.4.0 (2020-11-19)
------------------
* add storage_config_uri (`#493 <https://github.com/ros2/rosbag2/issues/493>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#535 <https://github.com/ros2/rosbag2/issues/535>`_)
* AMENT_IGNORE rosbag2_py for now (`#509 <https://github.com/ros2/rosbag2/issues/509>`_)
* rosbag2_py reader and writer (`#308 <https://github.com/ros2/rosbag2/issues/308>`_)
* Contributors: Karsten Knese, Mabel Zhang, Michael Jeronimo

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
* Make rosbag2 a metapackage (`#241 <https://github.com/ros2/rosbag2/issues/241>`_)
* Contributors: Anas Abou Allaban, Karsten Knese, Prajakta Gokhale

0.2.4 (2019-11-18 17:51)
------------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Load metadata from storage if no yaml file is found. (`#210 <https://github.com/ros2/rosbag2/issues/210>`_)
  Signed-off-by: Knese Karsten <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.2.3 (2019-11-18 13:55)
------------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* multifile reader (`#206 <https://github.com/ros2/rosbag2/issues/206>`_)
  * Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address commments:
  - Rewrite history
  - Move sequential reader implementation to header/source
  - Change namespaces
  - Linting
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Add visiblity control header
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address structural review feedback
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Remove extraneous newline
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Add new BaseReaderInterface
  * Add new reader interface
  * Use the interface in sequential reader
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Remove extra newline
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Final reader class implementation (`#4 <https://github.com/ros2/rosbag2/issues/4>`_)
  * final reader class
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * adaptations for rosbag2_transport
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * address review comments
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * Make BaseReaderInterface public
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Rebase on writer changes
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address commments:
  - Rewrite history
  - Move sequential reader implementation to header/source
  - Change namespaces
  - Linting
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Final reader class implementation (`#4 <https://github.com/ros2/rosbag2/issues/4>`_)
  * final reader class
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * adaptations for rosbag2_transport
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * address review comments
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * rebase
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * structurial changes for rosbag2
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * rosbag2_transport adaptations
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fixes for rebasing
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * pragma for windows
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * remove unused file
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * multifile sequential reader
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* writer interface (`#205 <https://github.com/ros2/rosbag2/issues/205>`_)
  * Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address commments:
  - Rewrite history
  - Move sequential reader implementation to header/source
  - Change namespaces
  - Linting
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Add visiblity control header
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address structural review feedback
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Remove extraneous newline
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Add new BaseReaderInterface
  * Add new reader interface
  * Use the interface in sequential reader
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Remove extra newline
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Final reader class implementation (`#4 <https://github.com/ros2/rosbag2/issues/4>`_)
  * final reader class
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * adaptations for rosbag2_transport
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * address review comments
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * Make BaseReaderInterface public
  Signed-off-by: Prajakta Gokhale <prajaktg@amazon.com>
  * Rebase on writer changes
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  Introduce new SequentialReader interface
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Address commments:
  - Rewrite history
  - Move sequential reader implementation to header/source
  - Change namespaces
  - Linting
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Final reader class implementation (`#4 <https://github.com/ros2/rosbag2/issues/4>`_)
  * final reader class
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * adaptations for rosbag2_transport
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * address review comments
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * rebase
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * structurial changes for rosbag2
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * rosbag2_transport adaptations
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fixes for rebasing
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * pragma for windows
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * remove unused file
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Rosbag splitting in Writer (`#185 <https://github.com/ros2/rosbag2/issues/185>`_)
  * Implement rosbag splitting in Writer
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Update unit tests for sqlite3 storage
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Update unit tests for rosbag2_tests
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add documentation
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Cleanup code
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add deleted test back in
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Append file extension only when SqliteStorage::open is passed READ_WRITE
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply formatting suggestions and throw in storage open when file exists with READ_WRITE
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add unit test for validating splitting in Writer
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Make reader use load_metadata and update tests
  Signed-off-by: Anas Abou Allaban <allabana@amazon.com>
  * Remove database_exists and make SqliteWrapper throw when database is bad
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Fix formatting and use relative_file_paths[0] from metadata
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Check if relative file paths is empty
  Signed-off-by: Anas Abou Allaban <aabouallaban@pm.me>
  * Update tests to reflect changes in reader
  Signed-off-by: Anas Abou Allaban <aabouallaban@pm.me>
* Contributors: Karsten Knese, Zachary Michaels

0.2.2 (2019-11-13)
------------------
* 0.2.2
  Signed-off-by: Michael Carroll <michael@openrobotics.org>
* (API) Generate bagfile metadata in Writer (`#184 <https://github.com/ros2/rosbag2/issues/184>`_)
  * Add support for specifying max bagfile size in storage_options
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add support for specifying max bagfile size in storage_options
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add helper functions in Writer required for bagfile splitting
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add helper functions in Writer required for bagfile splitting
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add get_identifier to io-interfaces
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Record metadata in Writer
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Record uri in Writer open
  * Accidentally removed this too early.
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions from PR
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add get_relative_path to BaseIOInterface
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add include on string to BaseInfoInterface
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Remove field init on test_writer
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Calculate bagfile size by summing all files
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Build BagMetadata inline
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Use std::min and std::max for metadata starting_time and metadata duration
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Moved storage->create_topic into if statement
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Applied suggestions
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Extracted init_metadata logic from Writer
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Reorder mocked methods to be alphasort
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Throw exception if erasing non-existing topic
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Throw if a topic fails to insert
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Added topic name to throw message when topic cannot insert
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Include topic name in exception when failed to removee a non-existing topic
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Include chrono
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Disable macros for min and max on windows
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Fix cmake linting error
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Update rosbag2/src/rosbag2/writer.cpp
  Co-Authored-By: Thomas Moulard <thomas.moulard@gmail.com>
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Update rosbag2/src/rosbag2/writer.cpp
  Co-Authored-By: Thomas Moulard <thomas.moulard@gmail.com>
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add unit tests for get_storage_identifier and get_relative_path
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Rename plugin_constants to test_constants
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Remove unused private field in TestReadOnlyPlugin
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
* Contributors: Michael Carroll, Zachary Michaels

0.2.1 (2019-10-23)
------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Add get_identifier to io-interfaces for support in bagfile splitting (`#183 <https://github.com/ros2/rosbag2/issues/183>`_)
  * Add support for specifying max bagfile size in storage_options
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add helper functions in Writer required for bagfile splitting
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add get_identifier to io-interfaces
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions from PR
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Moved database_exists in sqlite_storage to be a free function
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Change get_identifier in BaseIOInterface to get_storage_identifier
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
* Add bagfile splitting support to storage_options and Writer (`#182 <https://github.com/ros2/rosbag2/issues/182>`_)
  * Add support for specifying max bagfile size in storage_options
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add helper functions in Writer required for bagfile splitting
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Store max_bagfile_size when Writer is opened
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Uncrustify
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Apply suggestions from PR
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add ROSBAG2_STORAGE_PUBLIC to MAX_BAGFILE_SIZE_NO_SPLIT
  This should fix the issue on Windows
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Renamed private function in Writer to not end in `_`
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
* zero copy api (`#168 <https://github.com/ros2/rosbag2/issues/168>`_)
  * adopt to changes in rclcpp::subscription
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * use init/fini function from introspection_ts
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fix line length
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Change storage interfaces for bagfile splitting feature (`#170 <https://github.com/ros2/rosbag2/issues/170>`_)
  * Change storage interfaces for bagfile splitting feature
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Remove extra line in TestPlugin
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
  * Add documentation to get_bagfile_size
  Signed-off-by: Zachary Michaels <zmichaels11@gmail.com>
* Contributors: Karsten Knese, Zachary Michaels

0.2.0 (2019-09-26)
------------------
* 0.2.0
  Signed-off-by: Michael Carroll <michael@openrobotics.org>
* enable address sanitizers only on 64bit machines (`#149 <https://github.com/ros2/rosbag2/issues/149>`_)
  * enable address sanitizers only on 64bit machines
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * remove quotes to compare integers
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Export pluginlib to downstream packages (`#113 <https://github.com/ros2/rosbag2/issues/113>`_)
  Signed-off-by: Esteve Fernandez <esteve@apache.org>
* Add support for parsing middle module name from type (`#128 <https://github.com/ros2/rosbag2/issues/128>`_)
  * Add support for parsing middle module name from type
  Allows support for message types generated from both msg and idl files.
  Signed-off-by: David Hodo <david.hodo@is4s.com>
  * test fixups and default behavior
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * deprecate legacy type extraction and add new
  Signed-off-by: David Hodo <david.hodo@is4s.com>
  * use pragma to avoid deprecation in test
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: David Hodo, Esteve Fernandez, Karsten Knese, Michael Carroll

0.1.2 (2019-05-20)
------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
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
* Contributors: Karsten Knese, Sriram Raghunathan

0.1.1 (2019-05-09)
------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Karsten Knese

0.1.0 (2019-05-08)
------------------
* generate changelog
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Handle message type name with multiple namespace parts (`#114 <https://github.com/ros2/rosbag2/issues/114>`_)
  * Handle message type name with multiple namespace parts
  For now, it is okay to ignore the middle parts of the namespace, but this should be updated in the future.
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Update tests
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
  * Remove extra line
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* fix compilation against master (`#111 <https://github.com/ros2/rosbag2/issues/111>`_)
  * use refactored test messages
  Signed-off-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
  * partial update
  Signed-off-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
  * fix rsbag2_converter_default_plugins
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fix rosbag2_transport
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * fix rosbag2_tests
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * add wstring to introspection message
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * default initialize qos profile
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * avoid deprecated publish signature
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* fix logging signature (`#107 <https://github.com/ros2/rosbag2/issues/107>`_)
  Signed-off-by: Dirk Thomas <dirk-thomas@users.noreply.github.com>
* Compile tests (`#103 <https://github.com/ros2/rosbag2/issues/103>`_)
  * move process helper to test_common
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * use stdexcept for runtime error
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
  * always install include directories
  Signed-off-by: Karsten Knese <karsten@openrobotics.org>
* Contributors: Dirk Thomas, Jacob Perron, Karsten Knese

0.0.5 (2018-12-27)
------------------
* generate changelog
* Contributors: Karsten Knese

0.0.4 (2018-12-19)
------------------
* generate changelog
* Contributors: Karsten Knese

0.0.3 (2018-12-14)
------------------
* Play old bagfiles (`#69 <https://github.com/ros2/rosbag2/issues/69>`_)
  * GH-138 Move calculation of bag size
  - previously in rosbag2::Info
  - now in storage plugin
  * GH-130 Add rosbag2_bag_v2_plugins package
  -This package will contain storage and converter plugins
  * GH-131 don't build plugins on Windows
  * GH-129 Add function to be generated
  - massive if/else between all message types
  - will be generated similar to ros1_bridge plugin
  * GH-138 Write storage plugin for rosbag v2 bags
  * GH-138 Make sure that no attempt to create a converter is made when trying to read a rsbag v2 bag file
  * GH-138 Add play end-to-end test for rosbag v2 plugin
  * GH-138 Use cmake files to find ros1 packages
  - Use files from ros1_bridge via PkgConfig
  * GH-138 Add generator code
  * GH-141 Add initial version of vendor package
  * GH-141 Improve vendor package to build on Mac
  * GH-138 Cleanup CMakeLists
  * GH-141 Use unmanaged Instance of class-loader
  - managed instance somehow isn't available for gcc 6.3
  * GH-141 Reduce patch and copy new toplevle CMakeLists by hand
  * GH-141 Fix Shared Instance usage
  * GH-141 Improve maintainability of vendor package
  - Document what patches do and why changes are necessary
  - Load ros1 packages through cmake macro
  - Do not export ros1 packages via ament
  - use commit hash of current master which is more stable than using melodic-devel
  * GH-138 Link against rclcpp - necessary for ros1_bridge
  * GH-138 Avoid crash when trying to play v2 bags which contain unknown message types
  * GH-138 Add CLI -s <storage_id> option to ros2 bag info and use it in rosbag2::info
  - this allows ros2 bag info to work also when the yaml metadata file does not exsist
  - this is always the case for rosbag1 bagfiles
  - it could also happen for sqlite or other storage based bagfiles
  * GH-138 Add end-to-end info test for rosbag v2 files
  * GH-138 Add unit tests to rosbag_storage
  * GH-138 Add method to extract filename from path to FilesystemHelpers
  * GH-138 Add proper logging for topics which cannot be converted
  * GH-138 Improve finding dependencies of ros1
  * GH-141 Explicitly import transitive dependencies of vendor package
  * GH-138 Skip tests via ament if ros1 is not available
  * GH-133 First split of plugins
  * GH-133 Write serialized rosbag message
  * GH-133 Improve converter plugin
  - move generation templates outside of plugin folders as both
  plugins need it
  - use ros::serialization routines to deserialize the ros message
  * GH-133 Add plugin to be found by pluginlib
  * GH-133 Remove empty check in converter
  - With the rosbag_v2_converter_plugin, we don't need to treat
  rosbag_v2 storage any different
  * GH-133 Assert serialization format in unit tests for storage
  * GH-133 Delete superfluous include folder
  - Only needed if we want to link against the library
  * GH-133 get_all_topics_and_types returns only valid ros2 types
  - This is necessary as the information is used by rosbag2_transport
  - ros2 bag info still shows all topics and types
  - rosbag::View::getConnections() can return multiple connections corresponding to the same topic
  * GH-133 Improve end to end test
  - use a bagfile with messages not known to ros2
  * GH-133 Reformulate info message in case of missing ros1-ros2 mapping for a topic
  * GH-14 Find messages first
  * Explicitly print message when on Windows
  Co-Authored-By: Martin-Idel-SI <external.Martin.Idel@bosch-si.com>
  * GH-14 Refactor rosbag_storage vendor package
  - Improve toplevel CMakeLists
  - Put all patches into a resource subfolder
  * GH-14 Reflect renames of converter interfaces
  * GH-156 Workaround for path problems
  * GH-156 Add documentation for plugin
  * GH-156 Fix the pluginlib version to greater 2
  * GH-156 Prohibit CMake from declaring paths as system paths
  This switches the order of ros2 and ros1 directories
  resulting in build failures
  * GH-156 Prohibit system include paths for rosbag plugins
  This can lead to switching ros1 and ros2 include paths resulting
  in missing symbols as the wrong pluginlib gets included
  * GH-14 Split patches
  * make README more verbose
  * add plugin specific readme
  * more readme for bag_v2 plugin
* Contributors: Martin Idel

0.0.2 (2018-12-12)
------------------
* generate changelogs
* update maintainer email
* Contributors: Karsten Knese

0.0.1 (2018-12-11)
------------------
* generate CHANGELOG.rst
* Split converters (`#70 <https://github.com/ros2/rosbag2/issues/70>`_)
  * GH-134 Split converter interface into Serializer and Deserializer
  - Allow plugins which can only read or write
  - Most important example: plugin for old rosbags
  * GH-134 Switch to using serializer and deserializer in factory
  * GH-134 Add test for serializer plugin
  * GH-134 Try to load Serializer and Deserializer
  - When loading a serializer, try to load both serializer and converter
  - Similar for deserializers
  * GH-134 Fix e2e test after improving error message for missing converters
  * GH-134 Remove duplicate code in converter factory
  * GH-134 Change namespace of converter interfaces
  - adapt namespaces to folder structure
  - folder structure similar to rosbag2_storage
  * GH-134 Hide pluginlib import via pimpl
  - We want to use template functions that require the pluginlib import
  - The pluginlib import should not be exported (this creates issues with
  downstream packages)
  - Similar to the storage factory, use a pimpl
  * GH-134 Adapt documentation
  * Minor documentation updates
  Co-Authored-By: Martin-Idel-SI <external.Martin.Idel@bosch-si.com>
  * GH-134 Rename converter interface to drop "interface"
  - already visible from namespace
* GH-144 Add missing pop for warning pragma (`#68 <https://github.com/ros2/rosbag2/issues/68>`_)
* Fix master build and small renamings (`#67 <https://github.com/ros2/rosbag2/issues/67>`_)
  * GH-143 Fix master build after merge of PR 66
  - Detail: avoid | in regexp as this is not portable.
  * GH-143 Rename cpp_type_support to rmw_type_support
  * GH-143 rename ros2_message_t to introspection_message_t
* rename topic_with_types to topic_metadata
* use converter options
* GH-142 replace map with unordered map where possible (`#65 <https://github.com/ros2/rosbag2/issues/65>`_)
* Use converters when recording a bag file (`#57 <https://github.com/ros2/rosbag2/issues/57>`_)
  * GH-118 Make rosbag2::Writer use converters
  - Use converters in Writer::write() when input rmw serialization format is different from desired storage serialization format
  - Add new field in rosbag2::StorageOptions to keep track of the rmw format given by the user to store the message in
  * GH-118 Add --encoding option to ros2 bag record
  * GH-118 Associate to each topic its rmw_serialization_format
  - Add 'serialization_format' field to TopicMetadata
  - Add 'serialization_forat' column in 'topics' table in sqlite storage
  - Remove 'storage_format' from BagMetadata and use the TopicMetadata field directly, instead
  - the field 'rmw_serialization_format' has been moved from rosbag2::StorageOptions to rosbag2_transport::RecordOptions, because it's a topic property rather than a storage one.
  - Currently all topics in a bag file must have the same serialization format
  - The tests have been updated accordingly
  * GH-118 Fix tests after rebase
  * GH-118 Fix MockMetadataIO and use it in test_writer
  * GH-118 Fix Windows build and minor refactoring
  * GH-118 Add test for writer to check that error is thrown if converter plugin does not exist
  * GH-118 Add test to check that metadat_io\_ writes metadata file in writer's destructor
  * GH-118 Build Converter before opening the database in Writer::open()
  - This assures that if one of the converter plugins does not exist, the database is not created
  * GH-118 Add end-to-end tests to check graceful failure if converter plugins do not exists
  - Both a test for record and play has been added
  * GH-118 Rename 'encoding' CLI option to 'serialization_format'
  * GH-127 Write serialization format in database also when it's not specified at CLI level
  - Tests to check that the serialization format is written in the database have also been added.
  * GH-17 Add leak sanitizer to test
  - one of the main test goals can only be ssen by valgrind or sanitizers
  - enable leak sanitizer for gcc builds only (for now)
  * GH-137: Fix cdr converter plugin
  - update pluginlib descriptions file after several renames
  - fix export of missing includes folder
  * GH-137 Add integration test for cdr converter
  * GH-137 Fix superfluous printf
  * GH-137 It suffices to have only one converter test
  * GH-137 Minor refactoring for better readability of test
  N.B. This exposes an pre-existing memory leak (not fixed here).
  * GH-137 Fix memory leak of topic_name
  - topic_name member needs to be freed
  - provide a setter for convenience
  - Directly assigning a string literal in the test is not sufficient as
  this would be static memory that does not need to be freed.
  * GH-17 Allow disabling the usage of sanitizers
  This allows manual usage of valgrind.
  * GH-17 Fix renaming after rebase
  * GH-17 Small cleanups (addressing review comments)
* Renaming struct members for consistency (`#64 <https://github.com/ros2/rosbag2/issues/64>`_)
  * GH-118 Rename rosbag2_storage::TopicMetadata to TopicInformation and rosbag2_storage::TopicwithType to TopicMetadata
  - The former TopicWithTye struct will be enlarged to contain also the rmw serialization format relative to the topic. This is why the name 'TopicMetadata' is now better suited for it.
  * GH-17 Rename timestamp to time_stamp for consistency in types
  * Fix renaming of TopicWithType to TopicMetadata
  * formatting
  * pass by const ref
* Use converters when playing back files (`#56 <https://github.com/ros2/rosbag2/issues/56>`_)
  * GH-112 Open storage for reading handing in rmw_identifier
  * GH-113 Cleanup: better naming
  * GH-113 Introduce interface for StorageFactory to allow mocks in tests
  * GH-113 Add test for SequentialReader for using converters
  - Added mocks for storage and converters (and factories)
  * GH-113 Implement skeleton convert function
  - Use convert only if necessary (different input and output formats),
  converters are only loaded if really necessary.
  - Allocate_ros2_message is public to enable extensive tests for this function.
  - Helper function to get any typesupport by name
  - Helper function for empty ros2_message
  * GH-113 Implement allocate_ros2_message
  - Treats most messages already.
  - Some combinations of nested messages with arrays are still missing
  - Cleanup of DynamicArrayNested messages is failing
  - Main difficulty is the cleanup of the allocated ros2_message which
  needs to be done manually
  - The test_ros2_message is intended to be run with valgrind and there
  should be no leaks or problems with free!
  * GH-113 Fix DynamicArrayNested deallocation
  Swapping with empty container seems more stable than deleting the data
  pointer of the container.
  * GH-113 Add test for BoundedArrayNested deallocation
  * GH-113 Refactoring of deallocation code
  * GH-113 Fix string initialization in all types
  * GH-113 Fix vector<bool> initialization
  * GH-113 Add test for deallocation of topic name + Refactoring
  * GH-113 Minor refactoring of converter
  * GH-113 Make sure to throw an error if converters do not exist
  * GH-113 Delete superfluous imports
  * GH-113 Small fix for deleting vectors
  * GH-113 Fix build after rebase
  * GH-30 Minor refactoring
  - The TODO comments have been removed because they're no longer relevant: they have been discussed in the PR review
  * GH-30 Give an allocator as parameter to allocate_ros2_message()
  * GH-111 Add missing test dependencies for CDR converter test
  * GH-128 Extend message allocation test to also cover big strings
  - Big strings are not treated with small string optimization and need
  to be checked, too.
  * GH-128 Add tests for nested arrays
  * GH-128 always initialize vectors with a placement new
  * pass by ref
  * use new getter functions
  * consistent function naming
  *  uncrustify
  * GH-30 Fix windows build
  * use visibility macros on all functions
* Implement converter plugin for CDR format and add converter plugins package (`#48 <https://github.com/ros2/rosbag2/issues/48>`_)
  * GH-111 Add package for converter plugins
  * GH-111 Add CDR converter plugin
  * GH-111 Add test for more primitives types
  * GH-116 Fix cdr converter after rebase on new converters interface
  * GH-116 Use rmw_serialize/rmw_deserialize directly in converter and link against rmw_fastrtps_cpp
  * Fix converter package.xml
  * Fix clang warnings
  * GH-30 Change interface to the same convention as rmw\_(de)serialize
  * comply to new rcutils error handling API
  * use poco to load fastrtps
  * Update rosbag2_converter_default_plugins/src/rosbag2_converter_default_plugins/cdr/cdr_converter.cpp
  Co-Authored-By: Karsten1987 <karsten@osrfoundation.org>
* Display bag summary using `ros2 bag info` (`#45 <https://github.com/ros2/rosbag2/issues/45>`_)
  * Display bag summary using `ros2 bag info`
  * Improve process execution helper to handle the working directory
  * Use metadata filename in sqlite storage to determine database name
  * GH-109 Write metadata file on Windows by hand
  - On Windows, the process is killed hard and thus does
  not write its metadata file
  - Since this is an issue with the test setup that seems
  very hard to fix, for now we just write the metadata
  file on our own
  * Remove empty bag folder if record gets aborted and no files are created
  - For example is neither --all nor topics are specified or if a non exsisting storage plugin is specified
  * Fail gracefully if a runtime error occurs when trying to record or play
  - For example if the storage plugin specified by the user at record does not exist
  * Log error in case of failing when loading metadata, and minor refactoring
  * Add comment to version field
  * Allow rosbag2 info without yaml file
  Currently only supported on rosbag2 side:
  - Allow passing a storage identifier to rosbag2::Info()
  - If a yaml file exists, read info from yaml
  - If no yaml file exists and a storage identifier was passed
  open storage and read info directly
  * GH-7 Don't try to read database name from metadata file when opening with ReadWrite io_flag
  - This avoids the logging of a 'failed to read metadata' error when recording a new bag
  * GH-7 Rename 'storage format' into 'serialization format'
  -In this way it is not confused with the storage id (e.g. sqlite3)
  * GH-7 Improve failure conditions
  * GH-7 Cleanup of superfluous forward declarations
  * GH-7 Further improve exception handling
* Add entry point for converter plugins (`#47 <https://github.com/ros2/rosbag2/issues/47>`_)
  * GH-101 Add converter interface
  * GH-102 Create format converter factory
  * GH-103 Write documentation for converter plugin authors
  * GH-16 Adjust rosbag2 message type
  * GH-16 Change name of converter interface to include "serialization"
  - Easier to differentiate between storage format (e.g. sqlite)
  and serialization format (e.g. cdr)
  - Closer to naming in ros middleware
  * GH-16 Improve plugin development documentation
  - Also adapt to name changes
  * GH-16 Fix naming of SerializationFormatConverterFactory
* Extract recorder from rosbag2_transport, fix test naming (`#44 <https://github.com/ros2/rosbag2/issues/44>`_)
* Introduce rosbag2_transport layer and CLI (`#38 <https://github.com/ros2/rosbag2/issues/38>`_)
  * rosbag2_transport package with python interface
  * use cpp for python extension
  * use rosbag2_transport cpp API
  * use rosbag2_transport API in cli
  * linters
  * GH-25 Rename target librosbag2 to rosbag2
  CMake already prepends libraries with `lib`, so the old name resulted
  in `liblibrosbag2`
  * GH-21 Initial call of rosbag2.record() from rosbag2_transport
  * GH-21 Add missing copyright header
  * GH-21 Cleanup clang tidy issues
  * GH-21 Remove rclcpp dependency from rosbag2
  * GH-21 Wire rosbag play into CLI
  * GH-21 Add missing test_depend in rosbag2_transport package.xml
  * GH-21 Unify name of python import
  * GH-21 Enable -a in CLI, show help on wrong args
  * GH-85 Introduce topic and type struct for readability
  * GH-85 Do not export sqlite3 as dependency from default plugins
  - not referenced in header, therefore unnecessary
  * GH-85 Move rosbag2 except typesupport to rosbag2_transport
  * GH-85 Add rosbag2 wrapper
  * GH-85 Change signature of create_topic to take TopicWithType
  * GH-85 Use rosbag2 in rosbag2_transport
  - Don't link against rosbag2_storage anymore
  * GH-84 Cleanup package.xmls and CMakeLists everywhere
  * GH-21 Add missing init() and shutdown() in record
  * GH-85 Fix Windows build
  * GH-85 Add visibility control to rosbag2
  * GH-85 Cleanup and documentation
  * GH-87 Add test package rosbag2_tests
  * GH-87 [WIP] Add first working prototype of an end-to-end test
  * GH-87 Use test_msgs instead of std_msgs/String in end-to-end test
  * GH-87 Use SIGTERM instead of SIGKILL and refactor test
  * GH-87 Make end-to-end test work on Windows
  * GH-87 Fix uncrustify
  * GH-87 Refactor end-to-end test fixture
  * GH-21 Extend transport python module interface
  The python interface should accept all options that can be passed to rosbag2_transport
  * GH-87 Fix test fixture for Windows
  * GH-87 Refactor test fixture
  * GH-87 Separate record from play end-to-end test
  * GH-87 Make record end-to-end test work
  * GH-87 Publish before recording to create topic
  * GH-87 Fix record all on Windows
  * GH-87 Check for topics instead of all
  * GH-87 Wait until rosbag record opened database
  * GH-87 Delete directory recursively
  * GH-87 Delete directories recursively on Linux
  * GH-87 Reset ROS_DOMAIN_ID to protect against concurrent tests
  * GH-89 Make rosbag2 interfaces virtual and add explicit open() method
  This allows downstream packages (e.g. rosbag2_transport) to mock these
  interfaces in tests.
  * GH-87 Improve test and refactoring
  * GH-87 Minor refactoring to increase test readability
  * GH-87 Fix environmental variable behaviour on Mac
  * GH-87 Fix Windows build
  * GH-89 Use mock reader and writer in rosbag2_transport tests
  * GH-87 Add play end_to_end test
  * GH-87 Improvements of test
  * GH-87 Fix Windows build
  * GH-89 Cleanup: small documentation fixes.
  * GH-89 [WIP] Test if Writer and Reader work with class visibility
  * GH-87 Stabilize rosbag2_play test
  * GH-87 Minor refactoring of tests
  * GH-87 Rename end to end tests
  * add license agreement
  * GH-89 Simplification of writing to in-memory storage
  * GH-89 Stabilize transport tests
  * GH-87 Refactoring of tests
  - Extract temporary file handling
  - Extract subscription management
  * GH-87 Add pytest cache to gitignore
  * GH-87 Refactoring of play test
  - Extract Publisher manager
  * GH-87 Extract record test fixture for readability
  * GH-89 Refactor transport tests
  - Use subscription and publisher manager just as e2e tests
  - Use options in recording
  * GH-89 Use temporary directory fixture in sqlite tests
  * GH-89 Conform to naming standard for tests
  * GH-89 Prevent burst publishing of all messages
  - Improves test stability
  * GH-89 Improve play stability
  - Sometimes the first message is lost (discovery)
  * GH-25 Fix package.xmls
  * Consistently use project name in CMakeLists
  * Minor cleanup
  - make rosbag2_transport description more expressive
  - hide unnecessary methods in typesupport_helpers
  - fix incorrect logging in tests
  - minor cleanup
  * Change name of nodes in rosbag2_transport
  * Cleanup folder structure in rosbag2_storage and rosbag2_tests
  - use src/<package_name>/ and test/<package_name>/ folders everywhere
  - harmonises with all other packages
  - results in better header guards
  * Export sqlite3 dependency as package dependency
  * Create node in Rosbag2Transport always
  * Only hold one node in rosbag2_transport
  * Move all duplicate files to common package
  * Adapt namespacing in test commons package
  - use "using namespace" declaratives for tests
  - use package name as namespace
  * Replace "Waiting for messages..." message
  * GH-25 rename rosbag2_test_commons -> rosbag2_test_common
  * GH-25 Overwrite already existing test.bag when recording
  This is a temporary solution and will be handled properly once a
  file path can be passed via the cli.
  * GH-25 Cleanups
  - Log every subscription
  - move all dependencies onside BUILD_TESTING for rosbag2_test_common
  * fix cmake typo for test_common
  * Remove superfluous loop in rosbag2 transport
  * Delete superfluous test_msgs dependency
  * Add rclcpp to test dependencies
  - Apparently ament_export_dependencies does not work in rosbag2_test_common
  * Fix rosbag2 node test
  - Clock topic is no longer present on all nodes
  - Remove assumptions on foreign ros topics
  * Fix dependencies by exporting them explicitly
* Add correct timing behaviour for rosbag play (`#32 <https://github.com/ros2/rosbag2/issues/32>`_)
  * GH-69 Read storage content in a separate thread
  For now the publishing starts only after the reading is completly
  done. This should change aufter GH-68 is done and a thread-safe
  queue can be used instead of std::queue.
  * GH-71 Add integration test for timing behavior
  * GH-68 Introduce vendor package for shared queue
  - Download and install headers from moodycamel readerwriterqueue
  - Download and install headers from moodycamel concurrentqueue
  - Use readerwriterqueue in code to load and publish concurrently
  * GH-71 Retain time difference of messages when playing a bag file
  - The main (play) thread sleeps until the time for publishing the
  message is reached.
  - Using std::chrono time_point and duration for type-safe time
  arithmetic instead of rcutils time types.
  * GH-71 Improve stability of read test
  - Subscribers need to maintain a longer history if the messages are
  not consumed fast enough.
  * GH-71 Fix Classloader instance lifetime
  The Classloader instance needs to outlive all objects created by it.
  * GH-71 Extract playing code into a class of its own
  Reason: record and play have almost no common code but do the exact
  opposite with the storage and rclcpp.
  * GH-70 Do not link explicitly against std_msgs
  - only required in tests
  - this decreases the amount of packages needed for a clean build without tests
  * GH-70 Fix error message of storage
  * GH-70 Fix pluginlib/storage issue for recording
  * GH-71 Cleanup: variable naming
  * GH-70 Load storage continuously instead of as fast as possible
  - Only load if queue contains less than 1000 messages
  - Wait a millisecond before loading again once the queue is long enough
  * GH-70 Add options struct to allow specification of queue size
  * GH-72 Wait for messages to fill up
  * GH-74 Rename integration tests to play/record tests
  * GH-74 Use test_msgs in integration tests
  - gets rid of string_msgs dependency
  * GH-70 Rename is_not_ready to is_pending, use bulk reading to queue
  * GH-70 Harmonize storage_loading_future variable
  * GH-88 Read messages in order of their timestamps
  - Currently, we write sequentially in order of arrival time so
  reading in id order is fine
  - This may change at a later time and should not change the reading
  behaviour, i.e. we need to read in order of timestamps
  * Fix compiler error on Mac
  * GH-8 Fix: use correct ros message type in test
  * GH-8 Cleanup: minor code style fixes
  * GH-8 Refactor future usage in player
  Make the future a class member of player to avoid having to hand it
  into several functions which is difficult with a move-only type.
  * GH-8 Cleanup: remove verbose logging for every stored message
  * GH-8 Refactor rosbag2 interface
  Add an explicit overload for record without a topic_names argument to
  record all topics.
  * fix: call vector.reserve instead of default initalization
  * fix record demo
* Improve sqlite usage and test stability (`#31 <https://github.com/ros2/rosbag2/issues/31>`_)
  * GH-64 Rearrange default plugins build to use public headers
  Also already links write integration test against the default plugins.
  * GH-64 Remove after_write_action
  Query the underlying db directly in tests to determine the amount of
  recorded messages.
  * GH-64 Add convenience getter for single line SQL result
  * GH-64 Add visibility macros to enable linking on Windows
  * GH-64 Remove second sqlite exception class (it is not needed)
  * GH-64 Fix hanging rosbag2_read_integration_test
  * GH-64 Always log sqlite return code
  * GH-64 Improve opening of sqlite database
  - Always open db with threading mode multi-thread. This forbids
  sharing database connections across threads. Db access from multiple
  threads is possible when each thread uses its own connection.
  - Open sqlite db accordingly to given io flags. Readonly open works
  only with already existing database.
  - Set journal mode pragma to WAL (write ahead log) and synchronous
  pragma to NORMAL. This should yield good write performance.
  - Small fix: use .db3 as db name in tests.
  * GH-64 Fix package test dependencies
  * GH-64 Fix cppcheck error
  * GH-64 Fix asserting typesupport in test (varies on architectures)
  * Cleanup
  - consistently use const ref of string instead of string for function
  arguments
  - simplify package dependencies
  - minor formatting
  * Make play integration test compile on Mac
  * Fix sqlite_wrapper_integration_test
* Record all topics (`#30 <https://github.com/ros2/rosbag2/issues/30>`_)
  * GH-23 Get all topics from node and sanitize
  * GH-23 Move methods to node for better interface
  * GH-23 Use rmw_serialized_message_t consistently
  * GH-23 Improve santization of topics
  * GH-65 Introduce and use better logging macros
  * GH-23 Use publisher to serialized message directly
  * GH-23 Improve readability of sanitizing topics and types
  * GH-23 Allow to write all available topics
  * GH-23 Add test for record all
  * GH-23 Cleanup: add missing const ref to record interface
  * Cleanup for doxygen
  * Improve topic sanitization
  - correctly expand topic names using rcl
  - do not check type correctness (supposed to be done internally)
  * Pass topic_name by reference
* Record and play multiple topics (`#27 <https://github.com/ros2/rosbag2/issues/27>`_)
  * GH-61 Read topic directly from message when playing and allow to play multiple topics
  * GH-61 Add test for SqliteStorage and update old ones
  * GH-62 Extend function to poll for any number of specified topics
  * GH-62 Allow subscription to several topics
  * GH-61 Obtain the topic name directly from the database
  - Uses a JOIN instead of mapping the topic_id to the name in code
  * GH-61 Cache read row in result iterator
  This allows repeated dereferencing on same row without quering the
  database again.
  * GH-62 Change demo-record to allow specifying multiple topics
  * GH-62 Add test to write non-string topic + refactoring
  * GH-62 Add test for subscription to multiple topics
  * GH-62 Cleanup
  * GH-62 Simplify test setup
  * GH-61 Cleanup
  * GH-61 consolidate storage integration test
  * GH-62 Consolidate write integration tests
  * GH-61 enhance read integration test to check multiple topics
  * GH-62 Improve rosbag integration test
  * GH-62: Polish rosbag2_rosbag_node_test
  * GH-62 Fix cpplint
  * GH-62 Fix memory leak in rosbag helper
  * GH-62 Cleanup of subscriptions
  * GH-62 do not use flaky timers in rosbag2_write_integration_test
  * GH-62 Use rmw_serialize_message_t consistently in test helper classes
  * GH-73 Use test_msgs in read_integration_test
  * GH-26 Cleanup: fix alphabetic orderung
* Allow an arbitrary topic to be recorded (`#26 <https://github.com/ros2/rosbag2/issues/26>`_)
  * GH-52 Extend db schema to include topic meta data
  - Two table db layout (messages and topics)
  - Messages table references topics table but without foreign key for
  improved write performance
  - Create_topic must be called for every topic prior to storing a
  message of this topic.
  - Sqlite_storage caches all known topics
  - At least for now the type information is stored as a simple string.
  * GH-54 Make first rcl subscription prototype work
  * GH-54 find type name from topic
  * GH-54 Publish messages from database knowing only topic name and pass topic name by terminal
  * GH-54 Refactoring of typesupport helpers
  * GH-54 Use c++ typesupport
  * GH-54 Use cpp typesupport and rclcpp::Node for publisher
  * GH-54 Add raw subscription and use in rosbag_record
  * GH-54 Add Rosbag2Node and Rosbag2Publisher classes and use them in Rosbag2::play
  * GH-54 Rename Rosbag2Publisher to RawPublisher
  * GH-54 Minor refactoring of Rosbag2Node
  * GH-54 Extract and test waiting for topic into its own method
  * GH-54 Fix read integration tests and linters
  * GH-55 Refactor Rosbag2Node::create_raw_publisher()
  * GH-54 Add subscription method to rosbag node
  * GH-54 Keep subscription alive
  * GH-54: Extract subscription to correct class
  * GH-55 Change interface of raw_publisher to match subscriber
  * GH-54 Add test for rosbag node
  * GH-54 Unfriend rclcpp class
  * GH-54 Make test more robust
  * GH-54 Fix build
  * GH-54 Minor cleanup and documentation
  * GH-55 Minor refactoring + TODO comment
  * GH-54 Change dynamic library folder on Windows
  * GH-54 Fix build
  * GH-54 Add shutdown to test
  * GH-55 Add test helpers methods for usage in multiple tests
  * GH-55 Add new method to read all topics and types in BaseReadInterface and use it in Rosbag2::play
  * GH-55 Fix gcc and msvc
  * GH-54 Rename raw to generic in publisher/subscriber
  * GH-55 Check that topic and associated type in bag file are well defined before playing back messages
  * GH-54 Prevent unnecessary error message loading storage
  * GH-54 Fix memory leak
  * GH-54 stabilize node test
  * GH-55 Check if database exists when opening storage with READ_ONLY flag
  * GH-54 Minor cleanup of subscriber
  * GH-54 Wait a small amount of time to let node discover other nodes
  * Add logging to false case
  * GH-54 Catch exceptions and exit cleanly
  * Use rmw_serialized_message_t and rcutils_char_array_t consistently
  * GH-4 Refactoring for correctness
  - pass a few strings as const reference
  - throw error when no topics could be found
  * Improve error messages when loading plugins
  * alphabetical order
  * type_id -> type
* Use serialized message directly (`#24 <https://github.com/ros2/rosbag2/issues/24>`_)
  * Adapt new interface
  * Try to write and read rcutils_char_array_t BLOBs in sqlite
  * Add simple test for arbitrary char ptr
  * Refactor SqliteWrapper and add tests
  * Write and read actual timestamp from serialized message and add relative tests
  * Add SqliteStatementWrapper class and refactor SqliteStorage and SqliteWrapper
  * Refactor test fixture
  * GH-50 Assert message content in write_integration_test, and remove TODOs
  * GH-50 Remove sqlite_storage_plugin unit tests
  * GH-50 Refactor SqliteStatements and SqliteStorage
  * GH-50 Fix build after rebase
  * GH-50 Make has_next() method no more const
  * GH-52 Extend statement wrapper with a generic bind
  * GH-50 Refactor after rebase
  * GH-59 cleanup db interface
  - Remove virtual on methods as this was added only for unit tests. We
  decided to use only integration tests for the sqlite plugins.
  - Changes semantics of SqliteStatement: represents always a prepared
  statement if not null.
  - Ensures that a SqliteStatementWrapper cannot be copied and does not
  publicly expose its sqlite_stmt as this would cause memory corruption.
  * GH-59 Introduce general read interface for sqlite statements
  - Uses a std::tuple for row data
  - Exposes an iterator interface for the query result
  * GH-59 Cleanup: remove unused files
  * GH-59 make sqlite interface fluent
  * GH-59 move creation of serialized message to rosbag2_storage
  This is not storage plugin specific but will be needed by most (if
  not all) plugins.
  * Change rcutil_char_array_t to rmw_serialized_message_t in subscriber
  * Remove debugging output in test
* initial version of plugin based storage api (`#7 <https://github.com/ros2/rosbag2/issues/7>`_)
  * initial version of plugin based storage api
  * Add readable and writable storage interfaces
  * Fix build and uncrustify
  * Delete first storage interface proposal and adapt storage factory to new one
  * Modify test to work with new storage interfaces
  * Adapt sqlite3 plugin to new interface and extract rosbag2 part to own project
  * Adapt read() and write() methods signature
  * Prevent pluginlib from using boost
  * Add plugin development documentation
  * Remove Sqlite dependencies from rosbag2 tests
  * Add tests to rosbag2_storage_default_plugins
  * Add visibility control for Windows in rosbag_storage
  * Rename visibility_control.h to visibility_control.hpp
  * Cleanup CMakeLists in rosbag2_storage
  * Use void * instead of char * in rosbag_storage
  * Update plugin_description.xml and write() method
  * Introduce better logging using rcutils in rosbag_storage
  * Adapt interface and introduce better logging
  * Fix package.xml in rosbag2_storage
  * Add storage facade for plugins which are both readable and writable
  * Extract bag_info struct to own file
  * Change storage interface to have read/write access
  * Adapt copyright and use copyright linter
  * rosbag2 serialized message
  * remove colcon ignores
  * Add visibility to template specializations
  * Remove no longer necessary File install from CMakeLists.txt
  * Refactor storage_factory_impl.hpp
  * Minor refactoring
  * Add COLCON_IGNORE files to irrelevant projects
  * Fix Windows warning
  * Simpler class hierarchy without WritableStorage
  * Use exceptions instead of bool returns everywhere in interface
  * Change rosbag2_storage interface
  * storage interfaces
  * linters
  * a bit of refactoring
  * expose opening and closing
  * take messages as shared ptr
  * linters
  * rename to open, unique_ptr for pimpl
  * remove obsolete api
  * comply with new interfaces
  * change templated open to explicit open_ro and open_rw
  * Delete superfluous classes + polishing
  * Adapt SerializedBagMessage format
  * Let sqlite3 storage use new interface
  * Fix tests in rosbag2
  * Write and read only data
  * Replace creation of shared instance by unmanaged instance
  * Add pragma for windows
  * Add visibility control for Windows
  * Expose template definitions
  * Move const to better location
  * Replace strcpy
  * Delete superfluous methods
  * Use visibility control in rosbag2
  * Minor cleanup
  * test for nullptr when opening storage
* add visibility macro (`#22 <https://github.com/ros2/rosbag2/issues/22>`_)
* (demo, sqlite3) First working rosbag2 implementation (`#6 <https://github.com/ros2/rosbag2/issues/6>`_)
  * First implementation of writer
  * Extract storage interface and sqlite3 implementation
  * Add test for sqlite storage
  * Split main() and rosbag2::record()
  * Add close() method to Storage
  * Add getMessage() method and refactor test
  * Refactor SqliteStorage constructor and open()
  * Add linters
  * Fix uncrustify
  * Fix cpplint
  * Specify test working directory
  * Better error handling
  * Use gmock matchers for assertions
  * Add test fixture for SqliteStorage tests
  * Extract message retrieval in tests into separate method
  * Add integration test for rosbag2::record()
  * Add ignore files for empty packages
  * Introduce create() method and refactor open()
  * Use shared pointer of Storage instead of SqliteStorage
  * Remove getDatabaseHandle() method
  * Fix uncrustify
  * Improve storage interface and add storage factory
  * Remove need of sleep() from integration test by usage of std::future
  * Move deletion of test database from fixture constructor to destructor
  * Use sqlite3 directly in intergration test instead of own sqlite wrapper
  * Move rosbag2::record() into Rosbag2 class
  * Use the test name as database file name
  * Add build instructions to README
  * GH-37 Rename camelCase methods to snake_case
  * Use common test fixture
  * Add RAII wrapper for sqlite API
  * Mock away sqlite from sqlite_storage test
  * Use more reasonable assert
  * Add test
  * Add virtual destructor to WritableStorage
  * Use file_name instead of database_name in StorageFactory
  * Implement saving of test files in a tmp directory for linux/Mac
  * Try to implement saving of test files in a tmp directory for Windows
  * Write and use proper gmock SqliteWrappe mock
  * Refactor integration test and get rid of promise/future where possible
  * Throw exception in resource aquisition constructors
  * Make SqliteWrapper destructor virtual
  * Refactor test fixture and update SqliteWrapper mock
  * Fix warning when moving a temporary object
  * GH-38 Refactor integration test
  * GH-38 Get rid of superfluous string constructor in emplace_back()
  * GH-38 Assert also execute_query() argument in sqlite_storage_test
  * GH-38 add StorageFactory test
  * GH-38 Refactor rosbag2 Test Fixture
  * GH-40 Add first implementation of a rosbag reader and publisher
  * GH-40 Add StorageFactory test when reading non-existing file
  * GH-40 Fix uncrustify
  * GH-40 Minor cleanup of CMakeLists
  * GH-40 Wrap sqlite statements
  * GH-40 Remove superfluous import
  * GH-40 Use better include
  * GH-40 Add play integration test
  * GH-40 Fix Warning when moving a temporary object in reading
  * GH-40 Initialize database pointer to nullpointer
  * GH-40 Fix reader integration test
  * GH-40 Polish storage wrapper
  * Revert "GH-40: Wrap sqlite statements"
  * GH-38 Fix Test Fixture after rebase
  * GH-38 Refactor read_integration_test and refix Windows conversion warning
  * GH-38 Add StorageFactory test
  * Simplify storage factory test
  * GH-38 Try to fix flaky test
  * GH-38 Move rclcpp::shutdown() at the end
  * GH-41 Fix windows warning due to virtual explicit operator bool
  * GH-41 Use sqlite3 vendor package in rosbag2
  * GH-41 Stop linking tests to sqlite
  * GH-41 Fix test fixture on Windows
  * GH-41 Cleanup test fixture includes
  * GH-41 Print test database name
  * GH-41 Correctly determine temp dir on Windows
  * GH-41 Show error message on sqlite_open failure
  * GH-41 Actually create temp dir on Windows
  * GH-41 Fix bool conversion warning in VS2015 build
  * Fix CMakeLists.txt after rebase
  * GH-40 Implement workouround to fix flaky test
  * Update package.xml
  * Add gtest test dependencies to package.xml
  * GH-40 Move to sqlite3_storage_plugin folder
  - The separation into the intended structure and plugin apis is not
  there yet. However, most code will stay in the storage plugin for
  sqlite3 file.
  - Proper separation of this code into storage plugin and rosbag layer
  will be done in https://github.com/bsinno/aos-rosbag2/issues/5.
  * GH-40 Add TODO comments and small cleanup
* initial setup
* Contributors: Alessandro Bottero, Andreas Greimel, Andreas Holzner, Karsten Knese, Martin Idel

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_test_msgdefs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.2 (2023-07-14)
-------------------
* Don't crash when type definition cannot be found (`#1352 <https://github.com/ros2/rosbag2/issues/1352>`_)
* Contributors: Emerson Knapp

0.22.1 (2023-05-18)
-------------------

0.22.0 (2023-04-18)
-------------------

0.21.0 (2023-04-12)
-------------------
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`_)
  The intention of this PR is to move the message-definition-finding capability outside of rosbag2_storage_mcap, and allow any rosbag2 storage plugin to store message definitions.
* Contributors: james-rms

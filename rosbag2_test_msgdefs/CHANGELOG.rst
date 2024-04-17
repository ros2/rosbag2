^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosbag2_test_msgdefs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.1 (2024-04-17)
-------------------

0.26.0 (2024-04-16)
-------------------

0.25.0 (2024-03-27)
-------------------
* Implement service recording and display info about recorded services (`#1480 <https://github.com/ros2/rosbag2/issues/1480>`_)
* Contributors: Barry Xu

0.24.0 (2023-07-11)
-------------------
* Don't crash when type definition cannot be found (`#1350 <https://github.com/ros2/rosbag2/issues/1350>`_)
  * Don't fail when type definition cannot be found
* Contributors: Emerson Knapp

0.23.0 (2023-04-28)
-------------------

0.22.0 (2023-04-18)
-------------------

0.21.0 (2023-04-12)
-------------------
* rosbag2_cpp: move local message definition source out of MCAP plugin (`#1265 <https://github.com/ros2/rosbag2/issues/1265>`_)
  The intention of this PR is to move the message-definition-finding capability outside of rosbag2_storage_mcap, and allow any rosbag2 storage plugin to store message definitions.
* Contributors: james-rms

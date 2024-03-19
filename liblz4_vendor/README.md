# liblz4_vendor

Vendor package for the [LZ4](https://github.com/lz4/lz4) library.

The licensing on lz4 is complicated; see
https://github.com/lz4/lz4/blob/a3be31154f2e4c928ae6dcba9fe7e89e7678c74b/LICENSE
for details.  In short, the library itself (everything in the lib directory)
is BSD, while everything else in the package is GPLv2.

ROS 2 only wants to depend on the BSD-licensed portion, so the name of this
package is very specifically chosen to emphasize that we are only vendoring
the library.

# mcap_vendor

Vendor package for the [MCAP C++ library](https://github.com/foxglove/mcap).

## Versioning notes

This project abides by the versioning guidelines in the [developer guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Developer-Guide.html#versioning).

The `mcap_vendor` semantic version is intentionally decoupled from the `mcap` library version defined in [mcap/types.hpp](https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/types.hpp#L17). The `mcap` C++ library does not expose a stable ABI, so its version says nothing about ABI compatibility. The `mcap_vendor` package is intended
to offer a backwards-compatible ABI within a given major version.

> *Side-Note*: ROS 2 package versions defined in `package.xml` do not support additional labels or metadata after the patch version (eg. the `-alpha.1` in `1.2.3-alpha.1`).

* We make no promises about ABI compatibility across any version of `mcap_vendor` in the Rolling release. Therefore, the `mcap_vendor` major version in Rolling is always 0. Major or minor revisions of `mcap` should result in a minor version bump of `mcap_vendor`.
* In stable ROS 2 distributions, the `mcap_vendor` major version is set at some non-zero value.This major version should not be updated for the lifetime of that distribution, because that would indicate a change that breaks ABI compatibility.
* When a new ROS 2 distribution is cut from Rolling, a new major version `N` is chosen and the `mcap_vendor` version is set to `N.0.0`. 
* The `mcap` version built by `mcap_vendor` in a stable distribution should not change. Neccessary bug-fixes should be maintained as patch files in `mcap_vendor/patches`, and these patches should not modify the `mcap_vendor` ABI.

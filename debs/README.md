# Private binary dependencies

This directory can host `.deb` packages for private dependencies when source access is
not available.

For `robotnik_controllers`, prefer using `dependencies/repos/private.repos` when the
developer has repository access. Use a `.deb` package only as a binary fallback for the
matching Ubuntu, ROS 2 and architecture versions.

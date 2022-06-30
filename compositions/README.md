# Compositions
This package contains C++ source files and Bazel targets which compose one or more nodes together in the configuration
which they will run on the hardware.

The source code here defines the actual main() functions and cc_binary targets that will go on the hardware.  It also
contains any compositions used for development and testing.
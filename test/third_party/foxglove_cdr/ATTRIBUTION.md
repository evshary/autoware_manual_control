# @foxglove/cdr byte vectors

The `Foxglove` tests in `test/test_cdr.cpp` use CDR byte vectors lifted verbatim
from the MIT-licensed `@foxglove/cdr` package (v3.5.1):

- Source: https://github.com/foxglove/cdr (v3.5.1, commit `14426698ed003f79956bc49706c9f82c53089134`)
  - `src/CdrWriter.test.ts` (tf2_msgs/TFMessage; UTF-8 string length)
  - `src/CdrReader.test.ts` (tf2_msgs/TFMessage; rcl_interfaces/ParameterEvent)

They are a CDR implementation independent of rmw_cyclonedds, used here as a
non-rmw byte-level oracle (and, via the reader vectors, an external decoder
oracle: foreign-produced bytes -> values). All vectors are classic CDR_LE
(encapsulation `00 01 00 00`), matching this transport's DDS-CDR wire.

License text: see `LICENSE` in this directory.

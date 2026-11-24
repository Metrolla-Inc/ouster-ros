# Recorded Ouster packets for the smoke test

`OS-2-128-U1_v2.3.0_1024x10.pcap` and `.json` are the Ouster SDK's own test data, copied
unchanged from the SDK repository (https://github.com/ouster-lidar/ouster-sdk, Release-0.11.1,
commit `483206f0baeffbbc11ee0883d54a651a5216563b`, path `tests/pcaps/`). Until vendor release
`jazzy-0.15.1` they came from this repository's `ouster-ros/ouster-sdk` submodule; the vendor now
carries the SDK in-tree without its test data, so the two files live here.

One full frame of a real OS-2-128 on firmware 2.3.0 (1024x10, profile RNG19_RFL8_SIG16_NIR16,
64 lidar packets of 16 columns, 10 IMU packets) and the sensor's metadata in the legacy (flat)
JSON format. `docker/smoke/ouster_packets.py` pins both files by sha256:

- `.pcap` `66ef581f62f7cec8d8067efff38f46687672c6b39cdaa65095e795268d3362b5`
- `.json` `7e77f644a80e998a9c60ac67309200c2b299fcaf6a140c8bec209af14372334a`

Licence: the Ouster SDK's BSD 3-Clause licence, `LICENSE-ouster-sdk` (copied from the same
commit).

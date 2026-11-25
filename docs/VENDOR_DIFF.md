# Vendor update: ouster-ros 0.13 base to `jazzy-0.15.1`

Status (23 Nov 2026): **taken on branch `feat/jazzy-0.15.1`** (owner decision, 22 Nov 2026:
Ouster targets the vendor's Jazzy release `jazzy-0.15.1`). This file replaces the plan written
when the update was not yet taken (D3 part A, `docs/VENDOR_UPDATE_PLAN.md`).

## What was merged

| | Commit | Date | Notes |
|---|---|---|---|
| Old vendor base | `a46cda1` | 2025-11-06 | vendor `ros2` (0.13.x line), one commit after the split point `06e7e29` |
| Fleet tag `v1.0.0` | `541d632` | 2026-02-10 | old base + Metrolla's container changes; fleets pin `ouster-ros-driver:v1.0.0` |
| `ros2` before this change | `2bf426d` | 2026-11-22 | `v1.0.0` + the driver contract (ouster-ros #3) |
| **New vendor base** | **`73cf036`** | 2026-08-22 | tag `jazzy-0.15.1` = head of vendor branch `jazzy-devel` (also tagged `0.15.1`, `humble-0.15.1`, `kilted-0.15.1`, `rolling-0.15.1`, `release-0.15.1`); ouster-ros 0.15.1, Ouster SDK 0.16.2 **in-tree** (`ouster-ros/ouster-sdk`, no submodule) |

How: `git merge jazzy-0.15.1` into `ros2` (no rebase of the published branch). The vendor's
jazzy line split from `ros2` at `06e7e29`; the old base `a46cda1` is that commit plus "Add OpenCV
to library dependencys", which 0.15.1 supersedes (it links OpenCV through `OpenCV_LIBS`).

Conflicts and how they were resolved:

- `ouster-ros/package.xml`: kept the vendor's new `curl` and `spdlog` dependencies and Metrolla's
  `<build_depend>libopencv-dev</build_depend>`.
- `ouster-ros/CMakeLists.txt`: took the vendor's file. Metrolla's `cv_bridge` lines (`find_package`
  and in `ouster_ros_library_deps`, commit `7d9df72` "fix: workflow build") were a workaround for
  the old base's `OpenCV` entry in `ament_target_dependencies`; 0.15.1 links `${OpenCV_LIBS}`
  directly, so the workaround is not needed. `cv_bridge` stays a `package.xml` dependency (the
  vendor's) and is still installed by `Dockerfile.jazzy`.
- The `ouster-ros/ouster-sdk` submodule (SDK Release-0.11.1, `483206f`) became the vendor's in-tree
  SDK 0.16.2; `.gitmodules` is gone.

Metrolla's changes, all kept (`git diff jazzy-0.15.1 HEAD`): `Dockerfile.jazzy`, `entrypoint.sh`
(the v1.0.0 file plus the opt-in block), `docker-compose.yml`, `Dockerfile` edits, the
`sensor_hostname` launch argument in `launch/driver.launch.py`, `libopencv-dev` in `package.xml`,
the contract (`docker/`), tests, workflows (`build-and-push.yml`, `ci.yml`; the vendor's
`docker-image.yml` and PR template stay removed), docs, and the vendor code changes below.

Adaptations made for 0.15.1 in this change:

- `Dockerfile.jazzy` installs the in-tree SDK's build dependencies `libzip-dev`, `libssl-dev`,
  `pkg-config`, and its SDK check now looks for the in-tree copy (same past-bug test).
- The smoke test's recording (the SDK's own test data) is not in the in-tree SDK, so it lives in
  `docker/smoke/recording/` (same bytes, same sha256, SDK licence alongside).
- The contract leaves the sensor's time-sync wiring alone (below, item 2).
- The fake sensor speaks both HTTP APIs SDK 0.16 uses and reports a firmware version per run.

**Vendor code changes: two, each in its own commit.**

1. **New with this update: firmware before 3.2 crashed 0.15.1 on its first scan.** The point
   types' field tables (`sensor_point_types.h`) read the `WINDOW` field for the
   `RNG19_RFL8_SIG16_NIR16` (the default single-return profile), `RNG19_RFL8_SIG16_NIR16_DUAL`,
   `RNG15_RFL8_NIR8_DUAL` and `RNG15_RFL8_WIN8` profiles, but SDK 0.16 leaves `WINDOW` out of a
   `LidarScan` for firmware before 3.2 (`get_field_types`: "remove WINDOW if FW is < 3.2").
   `LidarScan::field()` threw in the scan processing thread and the process aborted: no point
   cloud in legacy mode (the launch process stays up, the node is gone after a few IMU messages),
   and the container exits in contract mode. Found by this change's first CI smoke run (fake
   sensor on 2.5.3 and 3.1.0); reproduced locally with the SDK built from this tree ("Field
   'WINDOW' not found in LidarScan" on 2.5.3 and 3.1.0, present on 3.2.0). Fix
   (`lidar_packet_handler.h`): a scan without `WINDOW` gets an empty one (there is no window data
   before FW 3.2), the way the vendor already adds its RGB fields to the same scans. Not fixed in
   the vendor's `ros2` head (`bbe9a89`) either; worth reporting upstream. Proof: past-bug test
   (source check, fails on the vendor's file) and the smoke runs on 2.5.3 and 3.1.0 (no WINDOW)
   and 3.2.0 (WINDOW from the packets).
2. **Kept from #3** (`~OusterDriver` stops the sensor connection thread before the node's
handlers and publishers are destroyed; `stop_sensor_connection_thread` protected) is **still
needed**: 0.15.1's `~OusterDriver` is unchanged and `~OusterSensor` still stops the thread only
after the derived members are gone. It merged without conflict; the past-bug test still checks it
and the smoke test still stops the driver five times under load.

## Differences that can change output or the sensor

Each item says what changed, who it affects (legacy = the image's own command, `ros2 launch ...
driver.launch.py`, as fleets run `v1.0.0`; contract = `METROLLA_DRIVER_CONTRACT=1`) and what proves
it. Fleets that pin `ouster-ros-driver:v1.0.0` are not affected by any of this: that image is not
rebuilt or moved.

1. **Firmware floor 2.4.0.** SDK 0.16 refuses a sensor whose `sensor_info.image_rev` is older
   than 2.4.0 (`SensorHttp::create`: "Please upgrade your sensor to FW 2.4 or later") before it
   sends anything. Old base: firmware 2.0 and later. Both modes. The driver logs the message and,
   in contract mode, keeps retrying every 5 s; nothing is written to the sensor. Proof: smoke
   step 5 (fake sensor on 2.3.0: the message, no configuration, no packets). **Needs hardware:
   the firmware of every fleet Ouster sensor must be read before any fleet moves to this image.**
2. **Settings written to the sensor at every start.** The old base wrote the UDP ports and
   destination, `operating_mode: NORMAL`, `azimuth_window`, and the lidar/timestamp mode and
   profile when set. 0.15.1 also always writes `signal_multiplier` (1), `phase_lock_enable`
   (false), `phase_lock_offset` (0), `nmea_ignore_valid_char` (false) and `nmea_leap_seconds` (0),
   and from its parameter file `multipurpose_io_mode` (OFF), `nmea_in_polarity` (ACTIVE_HIGH),
   `nmea_baud_rate` (BAUD_9600), `sync_pulse_in_polarity` (ACTIVE_HIGH) and
   `sync_pulse_out_polarity` (**ACTIVE_LOW**). The SDK only sends a configuration that differs
   from the active one, so a sensor already on these values is not touched; a site that set phase
   lock, a signal multiplier, NMEA or sync-pulse wiring on the sensor's web page has it reset.
   - Legacy: all of the above. On a sensor with the factory settings (as the fake sensor models
     them, from Ouster's sample configuration) this changes `sync_pulse_out_polarity` from
     ACTIVE_HIGH to ACTIVE_LOW. Proof: smoke step 2 lists every setting the driver changed and
     requires exactly the destination, the ports and `sync_pulse_out_polarity`.
   - Contract: renders the five string settings empty, which the node does not send
     (`is_arg_set`), so the sensor keeps its own time-sync wiring; the numeric ones above are
     still written (the node always sends them; leaving them alone needs a vendor code change,
     not made). Proof: unit `test_time_sync_wiring_is_left_to_the_sensor`; smoke steps 4a/4b
     require that nothing but the destination, ports (and the time mode with `TIME_MODE=ptp`)
     changed on the sensor.
   - Needs hardware: that factory defaults are as modelled, and what a real sensor on each
     firmware does with these writes.
3. **HTTP API.** SDK 0.16 reads `GET api/v1/sensor/config` first, then uses the
   `sensor/cmd/...` API on firmware before 3.1 and the staged-configuration API (`POST
   api/v1/sensor/config?staging=true&reinit=...`) on 3.1 and later; it reads `api/v1/user/data`
   on 2.5+ (not 2.4, 3.0). No output change; it decides which firmware works. Proof: the fake
   sensor implements both, checked locally with the SDK 0.16 client built from this tree against
   the fake on 2.4.0, 2.5.3, 3.0.1 and 3.1.0 (configure, metadata, full scans, IMU) and 2.3.0
   (refused); smoke runs the image on 2.5.3 (legacy, ptp, stops) and 3.1.0 (contract defaults).
4. **Per-point time `t` (stamps inside the cloud).** Old base: a destaggered point in output
   column `v` got the time of column `v`, not of the column it was measured in. 0.14+ (vendor fix
   "align timestamps"): the time of the measured column, `(v - pixel_shift[u]) mod width`. On the
   OS-2-128 recording 3 of 4 rows are shifted, so 75% of points get a different `t` (by up to 12
   columns, about 1.2 ms at 1024x10). Both modes. Message stamps (`header.stamp`) are unchanged:
   host receive time extrapolated to the scan start (`TIME_FROM_ROS_TIME`), the first column's
   time (sensor/gps), minus 37 s for PTP. Proof: smoke checks `t` of every point of every
   complete cloud it receives (at least two) against the recording (`--check-t`, legacy and contract runs); unit tests show the
   old rule fails that check; stamps checked as before (host within the window, PTP 37 s behind).
5. **Point layouts of other point types (vendor BREAKING).** `native`/`legacy` types narrowed
   fields (`Point_LEGACY` signal/near_ir uint16 to uint8, reflectivity uint32 to uint8;
   `RNG19_RFL8_SIG16_NIR16` reflectivity uint16 to uint8; `RNG15_RFL8_NIR8` reflectivity and
   near_ir to uint8) and fixed the order of FLAGS / PointLEGACY fields; new `xyzrgb`, `xyzrgba`,
   `color_point`. Not used: both modes publish `point_type: original` (`ouster_ros::Point`: x, y,
   z, intensity, t, reflectivity, ring, ambient, range), whose layout is unchanged. Proof: smoke
   decodes every field of every pixel; with the released image available the field list is
   compared with it.
6. **Units.** Range (mm, `range` field), xyz (m), reflectivity, signal (`intensity`), near-IR
   (`ambient`): unchanged, proven pixel by pixel. IMU: now from the SDK's `ImuPacket::accel()`
   (m/s^2, g x 9.80665) and `gyro()` (rad/s) instead of the node's own conversion: the same
   units and constants; covariances unchanged. Proof: smoke checks every IMU message's six
   values against the recorded packets converted (new), frame and stamp; checked locally with
   the SDK client too. New: with the `ACCEL32_GYRO32_NMEA` IMU profile (firmware 3.2) one packet
   gives several IMU messages; the legacy IMU profile gives one, as before.
7. **Frames and topics.** Unchanged: `os_sensor`, `os_lidar`, `os_imu` (or the contract's
   prefixed names), topics `points`, `points2`, `imu`, `telemetry`, `metadata`, `scan`, images,
   packets; static TF from the sensor's metadata. Proof: smoke (frame of clouds and IMU, topic
   names).
8. **Defaults in `driver_params.yaml`.** Every key of the old file keeps its value. New keys:
   `multipurpose_io_mode`, `nmea_*`, `sync_pulse_*` (item 2), `columns_per_packet` (0),
   `udp_profile_imu` (''), `imu_packets_per_frame` (0), `gyro_fsr`/`accel_fsr` (''),
   `operating_mode` (NORMAL), `signal_multiplier` (1.0), `phase_lock_*`,
   `lidar_frame_azimuth_offset` (-1), `return_order` (''), `bloom_reduction_optimization` (''),
   `min_distance` (-1); empty or -1 is not sent. Proof: unit test (every vendor key kept except
   the contract's), the contract goldens (regenerated deliberately in this change: only these
   lines differ).
9. **Metadata.** `/<ns>/metadata` and the metadata file are the JSON SDK 0.16 collects (adds
   `user_data` and an `ouster-sdk` block); a consumer that parses it must accept the extra keys.
   Not checked field by field (the smoke test checks the file is written).
10. **New capabilities, off by default**: FW 3.2/4.0 lidar profiles (`RNG15_RFL8_NIR8_DUAL`,
    `*_ZONE16`, `RNG15_RFL8_WIN8`, RGB16), varying columns per packet, tone-mapped RGB. Only
    reached when the sensor is set to those profiles (the contract keeps the sensor's profile).
11. **Removed launch files**: `sensor.independent.launch.py` and `.xml`. Nothing uses them (the
    image runs `driver.launch.py`; the repo's `docker-compose.yml` names `sensor.launch.xml`,
    which still exists).
12. **Crash on firmware before 3.2** (vendor 0.15.1 with SDK 0.16; fixed here, above). Without the
    fix every sensor on firmware 2.4-3.1 with the default RNG19 profile would publish no cloud.
13. **Build**: SDK built from the in-tree copy with `libzip-dev`, `libssl-dev`, `pkg-config`; the
    `cv_bridge` CMake workaround dropped (above). Proof: the image build in CI (amd64 and arm64).
14. **First clouds after start can be partial (unchanged from the old base).** The node opens its
    UDP socket and points the sensor at it before it starts reading (configure, then activate),
    and the vendor's `min_scan_valid_columns_ratio` is 0.0 in both the old and the new parameter
    file. So the first published scan can hold only what the socket buffer kept, and the next one
    only the part of a later frame that arrived after the driver caught up; missing columns are
    zeroed. In the smoke runs on CI: the first 17 of 64 packets (consistent with a buffer of
    2 x 212992 bytes, the kernel's default `net.core.rmem_max`; the SDK asks for 1 MiB), then the
    last 28 packets of a later frame, then complete scans. In that second scan every point's `t`
    is off, because with sensor time the driver imputes the scan start from the previous scan's
    last column as if no frame had been missed (vendor code, also in v1.0.0); the message stamp
    is affected the same way. A driver started mid-rotation on a real sensor, or one that misses
    whole frames, publishes such scans too. Both modes. Proof: smoke accepts partial clouds only
    before the first complete one (every column exact or empty, at most 3), then requires every
    cloud complete and exact, `t` included, and at least two of them;
    `tests/regression/test_smoke_check.py` reproduces the numbers of the CI runs (ros2 a13960b
    and this fix's first run). A consumer that needs whole scans can set
    `min_scan_valid_columns_ratio` (legacy parameter file; not a contract variable).

## Goldens

- Legacy goldens (`tests/legacy`) are those of the released v1.0.0 entrypoint and are unchanged;
  `entrypoint.sh` is still the v1.0.0 file plus the opt-in block.
- Contract goldens (`tests/regression/golden`): the five rendered-parameter goldens were
  regenerated because the vendor's parameter file gained keys (item 8); the contract's own lines
  differ only by the five empty time-sync settings (item 2). Refusal goldens unchanged.

## Release tag

`v0.15.1-metrolla.1` (pattern `v<vendor version>-metrolla.<n>`, owner 22 Nov 2026; the image
workflow refuses any other `v*` tag). Vendor version `0.15.1` rather than `jazzy-0.15.1`: the
vendor's `jazzy-0.15.1`, `humble-0.15.1`, `kilted-0.15.1`, `rolling-0.15.1`, `release-0.15.1` and
`0.15.1` tags are all the same commit, `package.xml` says 0.15.1, and the image is Jazzy-only by
its Dockerfile; `v0.15.1-metrolla.1` reads like the other driver forks' tags and does not reuse
the vendor's `0.15.1`. Tags are the owner's: none is pushed by this change.

## Weekly comparison (C22)

`.github/vendor-watch.json` lives on the default branch `master` (the schedule only runs there).
After this change merges into `ros2`, update it on `master` (a separate pull request, since this
one targets `ros2`): the `ouster-ros ros2` entry becomes branch `jazzy-devel`, tags `jazzy-*`,
`release-*`, base commit `73cf03684ae3576b40a263ae31bb64a617268aca` with "how" naming this merge;
the `ouster-sdk` entry is removed (the SDK is in-tree and comes with the vendor's branch). Until
then the weekly issue keeps saying the fork is behind its old base, which is no longer the base.

## Not proven (needs hardware)

Real firmware HTTP behaviour on 2.4, 2.5, 3.0, 3.1, 3.2 (the fake sensor follows the SDK's code,
not a sensor), the sensor's factory values for item 2 and its response to those writes, the
firmware of the fleet sensors (item 1), PTP and sync-pulse time on a real sensor, reconnect after
power loss, other models and modes (only OS-2-128 1024x10 `RNG19_RFL8_SIG16_NIR16` with the
legacy IMU profile is replayed), FW 3.2 IMU profile, dual return, RGB profiles. Certification
needs a real Ouster sensor.

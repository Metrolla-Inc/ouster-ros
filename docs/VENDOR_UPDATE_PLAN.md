# Bringing this fork to the vendor's current ROS 2 release: plan

Status (22 Nov 2026): **not done in this change; planned here.** The driver contract ships on the
current base (the code of fleet tag `v1.0.0`). Reason: the update is not safe without a real
sensor (below), and nothing in this repository can test it against one.

## Where the fork is

| | Commit | Date | Notes |
|---|---|---|---|
| Fork branch `ros2` = fleet tag `v1.0.0` | `541d632` | 2026-02-10 | vendor base + Metrolla's container changes |
| Vendor base (merge-base with vendor `ros2`) | `a46cda1` | 2025-11-06 | recorded in `.github/vendor-watch.json` on `master` (C22) |
| Ouster SDK submodule | `483206f` | 2024-05-15 | SDK Release-0.11.1 |
| Fork default branch `master` | | | the vendor's ROS 1 line, no Metrolla changes; not built by anything |

Metrolla's changes on top of the vendor base (`git diff a46cda1 541d632`): `Dockerfile.jazzy`,
`entrypoint.sh`, `docker-compose.yml`, `.github/workflows/build-and-push.yml` (replacing the
vendor's `docker-image.yml` and PR template), small `Dockerfile` edits, `cv_bridge` added to
`ouster-ros/CMakeLists.txt`, `libopencv-dev` to `package.xml`, and a `sensor_hostname` launch
argument in `launch/driver.launch.py`. No driver source change.

## The vendor's releases (checked 22 Nov 2026)

- Vendor branch `ros2`: newest tag **`ros2-v0.14.0`** (`f56b1a0`, 2026-02-26, "Upgrade to
  OusterSDK 0.16.x and support FW 3.2"), which is the **next commit after this fork's base**. The
  branch has 19 more untagged commits up to `bbe9a89` (2026-09-24).
- The vendor's newest release overall is **0.15.1** (tags `jazzy-0.15.1`, `humble-0.15.1`,
  `kilted-0.15.1`, `rolling-0.15.1`, `release-0.15.1`, all `73cf036`, 2026-08-22). They are on the
  distribution branches (`jazzy-devel` ...), which split from `ros2` at `06e7e29` (2025-09) and
  carry the SDK in-tree (ouster-sdk 0.16.2) instead of as a submodule. `ros2-v0.14.0` is not on
  that line; `0.14.1`/`0.14.2` are.
- The task named `ros2-v0.14.0`. It is still the newest tag on the branch this fork follows. For a
  Jazzy image the vendor's own current release is `jazzy-0.15.1`; which one to take is an owner
  decision (below).

## Why not now: what changes output or sensor state

From `git diff a46cda1 ros2-v0.14.0` and its CHANGELOG (0.14.0 section) and the SDK's CHANGELOG:

1. **Sensor firmware older than 2.4.0 is refused.** The SDK goes from 0.11.1 to 0.16.x; 0.16
   removed support for firmware older than 2.4.0 (`ouster_sensor/src/sensor_http.cpp`: `fw.major <
   2 || (fw.major == 2 && fw.minor <= 3)` throws). The fleet sensors' firmware is not recorded
   anywhere this task can read. A sensor on 2.0-2.3 would stop working after the update.
2. **Settings written to the sensor on every start.** 0.14.0 always sends `signal_multiplier`
   (1.0), `phase_lock_enable` (false) and `phase_lock_offset` (0) from the launch parameters, in
   addition to `operating_mode` (NORMAL, already sent by v1.0.0). A site that set phase lock or a
   signal multiplier on the sensor's web page would have it reset at every driver start.
3. **Point layouts (BREAKING in the vendor changelog).** Narrowed fields in the profile-specific
   point types (`Point_LEGACY` signal/near_ir uint16 to uint8, reflectivity uint32 to uint8;
   `Point_RNG19_RFL8_SIG16_NIR16` reflectivity uint16 to uint8; `RNG15_RFL8_NIR8` reflectivity
   and near_ir to uint8) and "correct the order of PointLEGACY fields". Fleets use `point_type:
   original` (`ouster_ros::Point`, unchanged), but a consumer that selected `native` or `legacy`
   would see a different field layout.
4. **Stamps.** "Correct the logic of scan_to_cloud_f when copying point timestamps" changes the
   per-point `t` field; `scan_col_ts_spacing_ns` now comes from `info.config.lidar_mode`.
5. **New lidar and IMU profiles** (FW 3.2: `RNG15_RFL8_WIN8`, `*_ZONE16`,
   `ACCEL32_GYRO32_NMEA`) and new parameters (`udp_profile_imu`, `imu_packets_per_frame`,
   `gyro_fsr`, `accel_fsr`, `lidar_frame_azimuth_offset`, `return_order`,
   `bloom_reduction_optimization`; empty or -1 = not sent).
6. **Removed launch files**: `sensor.independent.launch.py` and `.xml`. Nothing in the fleets uses
   them (they run `driver.launch.py` through the image's command).
7. The 1212-file SDK change (0.11.1 to 0.16) rewrites the HTTP client, packet parsing and scan
   batching the clouds come from.

Items 1 and 2 change what happens to a real sensor and can only be checked on one. Item 3 and 4
can be checked with recorded packets (the smoke test's fake sensor and pixel check already do this
for the current code).

## Trial merge (done locally, not pushed)

`git merge ros2-v0.14.0` into `ros2`: one conflict, `ouster-ros/package.xml` (Metrolla's
`<build_depend>libopencv-dev</build_depend>` next to the vendor's new `<depend>curl</depend>` and
`<depend>spdlog</depend>`: keep all three). Everything else merges cleanly, including Metrolla's
`cv_bridge` lines in `CMakeLists.txt` and the `sensor_hostname` launch argument.

## Plan

1. Owner decides the target: `ros2-v0.14.0` (what the fork follows, task text) or `jazzy-0.15.1`
   (the vendor's current Jazzy release, SDK in-tree; changes the vendor branch vendor-watch
   follows). Recommendation: `jazzy-0.15.1`, since the image is Jazzy and that line gets the
   vendor's release fixes; then follow `jazzy-devel` in vendor-watch.
2. Read the firmware of every fleet Ouster sensor (`http://<sensor>/api/v1/system/firmware`, on
   the bench or through the unit). Any below 2.4.0: upgrade the sensor first or stay on v1.0.0.
3. Branch from `ros2`, `git remote add vendor https://github.com/ouster-lidar/ouster-ros.git`,
   `git fetch vendor --tags`, merge the chosen tag (never rebase the published branch), resolve
   `package.xml` as above.
4. Contract: re-run `ouster_env.py selfcheck` (the image build does) - the vendor
   `driver_params.yaml` gains keys, which the contract keeps at the vendor's values; decide whether
   the contract should pin `signal_multiplier`/`phase_lock_*` or leave them out of the parameter
   file so the sensor keeps its own (needs a code change in the vendor node, since an unset
   parameter takes the declared default).
5. Smoke: the fake sensor must report firmware 2.4 or later (`fake_sensor.py` serves the
   recording's 2.3.0); the pixel check then proves the cloud from the same recorded packets is
   unchanged (range, reflectivity, signal, near-IR, ring, distance), and the legacy golden diff
   shows any default that moved.
6. Update `docs/` and `.github/vendor-watch.json` on `master`: `base` = the merged vendor commit,
   the SDK entry = the new submodule commit (or drop it for 0.15.1's in-tree SDK), `how` = this
   merge. Cut a new tag (never move `v1.0.0`); fleets keep `v1.0.0` until someone repins them
   after a bench run with a real sensor.

## Weekly comparison (C22)

`.github/vendor-watch.json` on `master` keeps base `a46cda1` and SDK `483206f`, which is still
true: nothing was taken from the vendor. The weekly run keeps reporting the fork behind (one open
issue), which is correct until step 6.

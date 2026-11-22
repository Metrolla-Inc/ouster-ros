# Metrolla driver contract (Ouster image)

The image `ghcr.io/metrolla-inc/ouster-ros-driver` (built from `Dockerfile.jazzy` on the `ros2`
branch) takes its settings from the environment the same way as the other Metrolla driver images
(Seyond, Hesai, LSLiDAR, Quanergy), and refuses a value it does not understand.

The contract is **opt-in**. Without `METROLLA_DRIVER_CONTRACT` (or with it empty or `0`) the image
behaves exactly as the released `v1.0.0` image that fleets pin: `entrypoint.sh` is the v1.0.0 file
with one block in front of it (tests/legacy pins this byte for byte), and the image's command is
unchanged.

## Switch

| `METROLLA_DRIVER_CONTRACT` | Behaviour |
|---|---|
| unset, empty, `0` | legacy: the v1.0.0 entrypoint runs the command (default: `ros2 launch ouster_ros driver.launch.py sensor_hostname:=192.168.1.201 viz:=false`) |
| `1` | contract: the environment below is checked, the parameter file rendered, the driver started |
| anything else | refused, exit 1 |

## Variables (with `METROLLA_DRIVER_CONTRACT=1`)

| Variable | Required | Accepted | Default | Becomes |
|---|---|---|---|---|
| `LIDAR_MODEL` | yes | `os0`, `os1`, `os2`, `osdome`, or with the channel count `os0-32`, `os0-64`, `os0-128`, `os1-32`, `os1-64`, `os1-128`, `os2-32`, `os2-64`, `os2-128`, `osdome-32`, `osdome-64`, `osdome-128`; any case, optional `ouster-` prefix, `-` and `_` ignored (so Ouster's own `OS-1-128`, `OS-DOME-128` work) | - | logged and written in the rendered file; the driver reads the actual model and calibration from the sensor |
| `SENSOR_IP` | yes | the sensor's IPv4 address (no hostname) | - | `sensor_hostname` |
| `SENSOR_ID` | no | `[a-z][a-z0-9_]{0,30}`, no `__` | none | ROS namespace `/<SENSOR_ID>` (default `/ouster`), and the prefix of every frame |
| `DATA_PORT` | no | 1024-65534 | `7502` | `lidar_port`; `imu_port` is `DATA_PORT + 1` (default 7503) |
| `DEVICE_PORT` | no | `80` only | `80` | nothing: the SDK always uses the sensor's HTTP API on port 80 |
| `HOST_IP` | no | IPv4, optional `/1`-`/32`; not the sensor's address; with a prefix the sensor must be in the subnet | none | `udp_dest` (where the sensor sends data). Unset: the sensor sends to the address the driver contacted it from (`set_udp_dest_auto`), as today |
| `FRAME_ID` | no | `[A-Za-z_][A-Za-z0-9_]{0,63}`, not the housing/IMU frame name | `os_lidar`, or `<SENSOR_ID>_os_lidar` | `lidar_frame` and `point_cloud_frame` (the cloud's `header.frame_id`) |
| `POINTCLOUD_TOPIC` | no | a ROS topic name, absolute or relative to the namespace | `points` (`/ouster/points`, or `/<SENSOR_ID>/points`) | a remap of the node's `points` topic |
| `TIME_MODE` | no | `host`, `sensor`, `gps`, `ptp` (any case); `ntp` refused | `host` | `timestamp_mode`: host = `TIME_FROM_ROS_TIME` (host receive time; the sensor's own time source is not changed), sensor = `TIME_FROM_INTERNAL_OSC`, gps = `TIME_FROM_SYNC_PULSE_IN`, ptp = `TIME_FROM_PTP_1588` (stamps = sensor time + `ptp_utc_tai_offset` -37 s). sensor/gps/ptp are written to the sensor |
| `PUBLISH_SCAN` | no | false values only | false | - |
| `OUSTER_RUN_DIR` | no | absolute path | `/tmp/ouster` | where `driver_params.yaml` and `metadata.json` are written |
| `DUAL_LIDAR` | no | false values only (empty = false) | false | - (one container per sensor) |

Fixed in contract mode (the rest of the vendor's `config/driver_params.yaml` is kept as it is):
`proc_mask: PCL|IMU|TLM` (no images, no raw packet topics, no LaserScan), `persist_config: false`
(nothing is saved on the sensor), `attempt_reconnect: true` every 5 s (the driver keeps trying while
the sensor is unreachable instead of sitting unconfigured), `pub_static_tf: true`
(`<sensor frame>` to lidar and IMU frames), `auto_start: true`, the sensor's own lidar mode and
packet profile (`lidar_mode` and `udp_profile_lidar` empty), `point_type: original`, organized and
destaggered clouds, best-effort (sensor data) QoS.

Topics (namespace `/ouster` or `/<SENSOR_ID>`): `points` (PointCloud2, or `POINTCLOUD_TOPIC`),
`points2` (second return, only with a dual-return profile on the sensor; not remapped), `imu`,
`telemetry`, `metadata`; services `get_metadata`, `get_config`, `set_config`, `reset`.

Frames: without `SENSOR_ID` `os_sensor`, `os_lidar` (or `FRAME_ID`), `os_imu` (the vendor's
names); with `SENSOR_ID=east` `east_os_sensor`, `east_os_lidar` (or `FRAME_ID`), `east_os_imu`.

Refusals: every problem is one line on stderr starting `ouster-driver: refusing to start:`, all
problems at once, exit 1. A legacy or vendor name (`SENSOR_PORT`, `SENSOR_TYPE`,
`SENSOR_HOSTNAME`, `LIDAR_PORT`, `IMU_PORT`, `UDP_DEST`, `LIDAR_IP`, `TIMESTAMP_MODE`,
`OUSTER_NS`, `LIDAR_MODE`, `UDP_PROFILE_LIDAR`, `PROC_MASK`, `PARAMS_FILE`) is refused with the
contract's name; an unknown variable under the contract's prefixes is refused with the closest
contract name. Other variables (`ROS_*`, `RMW_*`, ...) pass through to the node.

Commands with the switch on: none (or the image's default command) = run; `check` prints the
rendered file and the exec line; `check --json` the same as JSON; any other command is refused.

## How it runs

`/entrypoint.sh` execs `/opt/metrolla/ouster/contract-entrypoint.sh`, which sources ROS 2 Jazzy and
the workspace and execs `python3 -I /opt/metrolla/ouster/ouster_env.py`. That checks the
environment, writes `<OUSTER_RUN_DIR>/driver_params.yaml` (the vendor file with the contract's keys
replaced; each must exist exactly once or the image build fails) and execs
`/opt/ros2_ws/install/ouster_ros/lib/ouster_ros/os_driver` with `--params-file` and
`auto_start: true`. The node is PID 1 and ends cleanly on SIGTERM (rclcpp's handler), exit 0. No
launch file runs: the vendor's `driver.launch.py` only drives the same configure and activate
transitions that `auto_start` requests.

## For the product (metrolla-edge generator)

```yaml
image: ghcr.io/metrolla-inc/ouster-ros-driver:<tag from this branch, pinned by digest>
user: "97000:97000"
read_only: true
tmpfs: [/tmp]
network_mode: host
ipc: host
cap_drop: [ALL]
security_opt: ["no-new-privileges:true"]
environment:
  METROLLA_DRIVER_CONTRACT: "1"
  LIDAR_MODEL: os1-128            # metrolla-edge model "ouster-os1-128" without the vendor prefix (or with it)
  SENSOR_IP: 192.168.1.201
  SENSOR_ID: <sensors[].id>
  DATA_PORT: "7502"               # IMU on DATA_PORT + 1; two sensors on one host need different ports
  FRAME_ID: <frame>               # optional
  POINTCLOUD_TOPIC: <topic>       # optional
  TIME_MODE: host                 # or ptp when the site has PTP
# no command, no config file mount
```

Host firewall: allow UDP `DATA_PORT` and `DATA_PORT + 1` in from the sensor, TCP 80 out to the
sensor.

## Not proven (needs hardware)

Certification needs a real sensor. Proven in CI only against a fake sensor that replays a real
OS-2-128 recording (firmware 2.3.0, from the Ouster SDK's test data) and answers the SDK's HTTP
calls: see TESTING.md. Not proven: a real sensor's HTTP API behaviour on each firmware (2.0 TCP
API, 2.1/2.2 variants, 3.x), `set_udp_dest_auto` on a multi-homed host, PTP and sync-pulse time
on a real sensor (the TAI-UTC offset), `attempt_reconnect` after a real power loss, packet loss at
2048x10 / dual-return profiles, the OSDome, OS0 and OS1 geometry (only OS-2-128 packets are
replayed), the factory network settings (DHCP, link-local, `os-<serial>.local`).

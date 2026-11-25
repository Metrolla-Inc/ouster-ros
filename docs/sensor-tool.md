# sensor-tool (Ouster)

`/usr/local/bin/sensor-tool` in the driver image (`docker/sensor_tool.py`, standard library
Python) reads an Ouster sensor's own settings and sets them: its address, where it sends its
data, and its clock source. It implements version 1 of the `sensor-tool` interface that
`metrolla-hostd`'s `sensor.program` runs (metrolla-edge, `docs/sensor-program.md`). The driver
never runs it; the host runs it as its own container with `--entrypoint sensor-tool`.

**Status: tested against a fake sensor only (`docker/smoke/fake_sensor.py` on loopback). No
command here has been run against a real sensor.** See "Not proven" below before trusting a write.

## Commands

```
sensor-tool probe    --ip A [--model M] [--timeout S] [--driver-stopped]
sensor-tool get-net  --ip A [--model M] [--timeout S] [--driver-stopped]
sensor-tool set-net  --ip A --new-ip B --netmask M --gateway G [--no-reboot] [--dry-run] [--verify-timeout S] --driver-stopped
sensor-tool set-dest --ip A --dest-ip H --data-port P            [--dry-run] [--verify-timeout S] --driver-stopped
sensor-tool set-ptp  --ip A --transport L2|UDPv4 --domain N --delay-mechanism E2E|P2P --timescale UTC|TAI
                     [--profile 1588v2] [--clock-source ptp]     [--dry-run] [--verify-timeout S] --driver-stopped
```

It prints exactly one JSON object on standard output, also for a usage error.

| Outcome | Exit | Meaning |
|---|---|---|
| `reported` | 0 | a read |
| `set`, `unchanged` | 0 | written and read back; or already right, nothing sent |
| `changes` | 2 | a dry run that would change something |
| `refused` | 1 | nothing was sent that changes the sensor: a bad argument, a write without `--driver-stopped`, an address the host could not reach afterwards |
| `failed`, `no-answer` | 1 | the sensor refused or another sensor answers at the new address; nothing answers |
| `unsupported` | 3 | the whole command cannot be done on this sensor; nothing was sent |
| `unverified` | 5 | a change was sent and the sensor did not answer with the new values in time; `answers_at` says where it answers |

A setting that cannot be set, while the rest of the command can, is listed in `unsupported`
(`{"setting": "net.gateway", "reason": "…"}`) and the outcome is that of the rest (`set` or
`unchanged`).

## What each command does on an Ouster

The whole interface is the sensor's HTTP API on TCP port 80. Reads and dry runs work beside the
running driver and need no `--driver-stopped`; every write needs it, and is refused without it
before anything is sent.

| Command | What is sent | Notes |
|---|---|---|
| `probe`, `get-net` | `GET api/v1/sensor/metadata/sensor_info`, `GET api/v1/sensor/cmd/get_config_param?args=active`, `GET api/v1/system/network`, `GET api/v1/time/ptp` | a sensor that does not answer the last two is read without them (netmask, MAC, PTP profile and state are then `null`, with a warning) |
| `set-net` | `PUT api/v1/system/network/ipv4/override` with the body `"<new address>/<prefix>"` | the netmask goes in the prefix. **The gateway cannot be set**: one other than `0.0.0.0` is listed in `unsupported`. An address the sensor has without an override (DHCP, link-local) is made static (`net.static`). No restart is sent and `--no-reboot` changes nothing. The read-back is at the new address, and the serial number must be the same |
| `set-dest` | the active configuration with `udp_dest`, `udp_port_lidar` and `udp_port_imu` (data port + 1) changed, staged whole with `set_config_param?args=.+<JSON>`; `reinitialize`; `save_config_params` | the data port is at most 65534 |
| `set-ptp` | `timestamp_mode: TIME_FROM_PTP_1588` the same way; and, when the transport or the delay mechanism differs, `PUT api/v1/time/ptp/profile` | UDPv4 + E2E is `default`, L2 + P2P is `gptp`, L2 + E2E is `default-l2-relaxed`. A profile that already has the transport and delay asked for is kept (`automotive-slave` for L2 + P2P). `unsupported`: a domain other than 0; P2P over UDPv4 (E2E is set); transport and delay on a sensor that does not show its profile. The timescale is the grandmaster's, not a sensor setting |

`get-net` reports `ptp.clock_source` as `ptp`, `internal` or `sync-pulse`, the firmware's own
profile name as `ptp.sensor_profile`, and `ptp.domain` as `null` (no call reads it).

## Rules, each with a test (`tests/unit/test_sensor_tool.py`)

- Reading and writing are separate code. On this sensor a GET can be a write, so the line is
  drawn by path: `read` sends a GET to one of four fixed paths; the five write paths go through
  `Writer`, which cannot be made in a dry run or without `--driver-stopped`. No other path can be
  sent: no DELETE of the address, no `set_udp_dest_auto`, no firmware or restart call.
- Every write needs `--driver-stopped`.
- A set command reads first, changes only what differs, reads back, and is idempotent.
- One connection at a time, counted in `sessions` (`{"http": 4, "max_concurrent": 1}`).
- What cannot be set is said.

## Sources

From the Ouster SDK this repository builds (0.16.2, in the tree at `ouster-ros/ouster-sdk`;
`sensor_http_imp.cpp`, `sensor_http.cpp` and `client.cpp` are in `ouster_sensor/src`, the others
in `ouster_client/src`); a unit test checks the cited lines still say this:

| Fact | Source |
|---|---|
| `GET api/v1/sensor/metadata/sensor_info`; `get_config_param?args=active` (firmware before 3.1) | `sensor_http_imp.cpp:46-49`, `53-58` |
| `set_config_param?args=<key>+<value>` answers `"set_config_param"`; `reinitialize` and `save_config_params` answer `{}`; all three are GETs (firmware before 3.1) | `sensor_http_imp.cpp:69-76`, `157-160`, `171-174`, `346-355` |
| the active set is staged whole, then `reinitialize`, then `save_config_params` to persist | `client.cpp:309-394` (`set_config`), lines 377-390 |
| `udp_ip` beside `udp_dest` on firmware that has it | `client.cpp:361-363`; `metadata.cpp:319-324` |
| the keys `udp_dest`, `udp_port_lidar`, `udp_port_imu`, `timestamp_mode` | `metadata.cpp:315-336`, `types.cpp:460-490` |
| `TIME_FROM_PTP_1588` and the other clock sources | `types.cpp:46-50` |
| `prod_line`, `prod_sn`, `image_rev` | `sensor_info.cpp:297-301` |
| `GET api/v1/system/network`; `PUT api/v1/system/network/ipv4/override` with a JSON string, the address "in CIDR form" | `sensor_http_imp.cpp:151-153`, `357-364`; `ouster_sensor/include/ouster/sensor_http.h:330-341` |
| the IMU port is the data port + 1 | this repository's contract, `docker/ouster_env.py` |

**Not from the SDK**: the shape of the answer of `GET api/v1/system/network`, how a sensor
behaves after the override, and everything of PTP beyond `timestamp_mode` (`GET api/v1/time/ptp`,
`PUT api/v1/time/ptp/profile`) are from Ouster's firmware user manual (HTTP API v1) **as
recalled**. The manual is not in this repository.

## Not proven, and limits

Nothing here has met a real sensor. In particular a real sensor must confirm:

- **Firmware 3.1 and later.** The tool reads and writes the configuration with the `sensor/cmd`
  calls, which the SDK uses only before firmware 3.1; from 3.1 the SDK uses
  `api/v1/sensor/config` (`sensor_http_imp.cpp:19`, `53-63`, `69-89`, `157-179`), and the tool
  does not. Whether a sensor on 3.1 or later still answers the `sensor/cmd` calls is not known
  here. The fake sensor of this repository does not (404) when it reports such a firmware: every
  command of the tool then ends `no-answer` ("did not give its configuration") and nothing is
  written. The unit tests and the smoke test run the tool against firmware 2.3.0 and 2.5.3.
- **Every manual path above**: that it exists on the firmware in use, the shape of its answer
  (`ipv4.addr`, `ipv4.override`, `ethaddr`; `profile`, `port_data_set.port_state`), and that a
  PUT takes a JSON string.
- **The body of the override.** The tool sends `"<address>/<prefix>"`, the SDK's form
  (`set_static_ip`). Whether the sensor applies it without a restart and keeps it over a power
  cycle is not confirmed. A sensor that refuses the body ends the run `failed`; one that needs a
  power cycle ends it `unverified` (there is no restart call here).
- **The gateway.** The tool sets none. The SDK can on firmware 3.2 and later (the override as an
  object with `addr` and `gateway`, `sensor_http_imp.cpp:366-385`); the tool does not send that
  form.
- **The PTP profile table.** What `default`, `gptp`, `automotive-slave` and `default-l2-relaxed`
  mean (transport, delay mechanism, domain 0) is read from the manual as recalled and from the
  profiles' names; `default-l2-relaxed` as "L2 with end-to-end delay" is from its name alone. The
  tool reports `ptp.transport` from this same table, so the host's own re-verify cannot catch a
  wrong table: only a sensor that locks to a grandmaster can.
- **`save_config_params` saves the whole active configuration**, not only the changed keys: what
  the driver set at its last start (lidar mode, ports, destination) is saved with it.
- **`unchanged` compares the active configuration only.** Whether a value is also saved cannot be
  read (the SDK has no call), so a value the driver set without saving reads as already right and
  is not saved by the tool. The driver sets the destination, ports and time mode again at every
  start anyway.
- A sensor moved by `set-net` is put back by another `set-net`; the tool never deletes the
  override, so a sensor that was on DHCP stays static.
- Firmware before 2.2 (no `sensor_info` path) is answered `no-answer`. The tool does not check
  the firmware otherwise; the driver in this image needs 2.4 or later (`sensor_http.cpp:57-62`).

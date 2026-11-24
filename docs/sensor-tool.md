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

From the Ouster SDK this repository builds (the submodule `ouster-ros/ouster-sdk` at commit
483206f, directory `ouster_client/src`); a unit test checks the cited lines still say this:

| Fact | Source |
|---|---|
| `GET api/v1/sensor/metadata/sensor_info`; `get_config_param?args=active` | `sensor_http_imp.cpp:18-20`, `36-38` |
| `set_config_param?args=<key>+<value>` answers `"set_config_param"`; `reinitialize` and `save_config_params` answer `{}`; all three are GETs | `sensor_http_imp.cpp:28-34`, `69-75`, `92-98` |
| the active set is staged whole, then `reinitialize`, then `save_config_params` to persist | `client.cpp:296-371` (`set_config`), lines 351-367 |
| `udp_ip` beside `udp_dest` on firmware that has it | `client.cpp:341-343`; `types.cpp:612-616` |
| the keys `udp_dest`, `udp_port_lidar`, `udp_port_imu`, `timestamp_mode` | `types.cpp:612-628`, `794-804` |
| `TIME_FROM_PTP_1588` and the other clock sources | `types.cpp:50-54` |
| `prod_line`, `prod_sn`, `image_rev` | `sensor_info.cpp:254-259` |
| the IMU port is the data port + 1 | this repository's contract, `docker/ouster_env.py` |

**Not from the SDK**, which has no call for the address or for PTP beyond `timestamp_mode`:
`GET api/v1/system/network`, `PUT api/v1/system/network/ipv4/override`, `GET api/v1/time/ptp` and
`PUT api/v1/time/ptp/profile` are from Ouster's firmware user manual (HTTP API v1) **as recalled**.
The manual is not in this repository.

## Not proven, and limits

Nothing here has met a real sensor. In particular a real sensor must confirm:

- **Every manual path above**: that it exists on the firmware in use, the shape of its answer
  (`ipv4.addr`, `ipv4.override`, `ethaddr`; `profile`, `port_data_set.port_state`), and that a
  PUT takes a JSON string.
- **The body of the override.** The tool sends `"<address>/<prefix>"`. Whether the sensor takes
  the prefix form, applies it without a restart, and keeps it over a power cycle is not
  confirmed. A sensor that refuses the body ends the run `failed`; one that needs a power cycle
  ends it `unverified` (there is no restart call here).
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
- Firmware before 2.2 (read by other paths, or over TCP, in the SDK: `sensor_http.cpp:30-51`) is
  answered `no-answer`.

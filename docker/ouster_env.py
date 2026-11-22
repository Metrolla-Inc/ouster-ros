#!/usr/bin/env python3
"""Environment contract of the Metrolla Ouster driver image (opt-in: METROLLA_DRIVER_CONTRACT=1).

Reads the container's environment, refuses to start on any value it does not understand, renders
the driver's parameter file from the image's own config/driver_params.yaml (the vendor's file, so
every setting the contract does not name keeps the vendor's default) and (subcommand ``run``)
execs the ``os_driver`` node directly, with ``auto_start`` on, so the node is PID 1 and receives
SIGTERM itself (rclcpp ends the node cleanly on SIGINT and SIGTERM).

The legacy image runs ``ros2 launch ouster_ros driver.launch.py``, which drives the node's
lifecycle (configure, activate) from the launch file. The contract uses the node's own
``auto_start`` parameter instead (os_sensor_node.cpp), which requests the same two transitions.

Standard library only: it runs in the image before the driver and in the unit and regression tests
outside the image, where there is no ROS.

Subcommands:
  run            validate, write the parameter file, exec the driver (the default)
  check [--json] validate and print what ``run`` would write and exec; nothing is started
  selfcheck      for the image build: the vendor parameter file renders, the driver binary exists

Exit codes: 0 ok; 1 refused (a value this image does not understand, nowhere to write, or an
unknown command).
"""

from __future__ import annotations

import difflib
import ipaddress
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Mapping

PROG = "ouster-driver"
EXIT_OK = 0
EXIT_REFUSED = 1

# Paths inside the image (Dockerfile.jazzy: ROS2_WS=/opt/ros2_ws, COPY . src/, colcon build).
WORKSPACE = "/opt/ros2_ws"
PACKAGE_SHARE = WORKSPACE + "/install/ouster_ros/share/ouster_ros"
VENDOR_PARAMS = PACKAGE_SHARE + "/config/driver_params.yaml"
DRIVER_NODE = WORKSPACE + "/install/ouster_ros/lib/ouster_ros/os_driver"
NODE_NAME = "os_driver"
DEFAULT_NAMESPACE = "ouster"  # driver.launch.py's ouster_ns default: topics /ouster/points ...
DEFAULT_RUN_DIR = "/tmp/ouster"
DEFAULT_ROS_HOME = "/tmp/ros"

# Ouster's factory defaults for the host-side UDP ports (config_params udp_port_lidar 7502 and
# udp_port_imu 7503). The contract always sets both, so the sensor sends to fixed ports a
# firewall can allow; the IMU port is DATA_PORT + 1.
DEFAULT_DATA_PORT = 7502
MIN_PORT = 1024  # the driver runs as a non-root uid with no capability: no port below 1024
MAX_DATA_PORT = 65534  # DATA_PORT + 1 must be a port too
# The SDK reaches the sensor at http://SENSOR_IP/api/v1/... : port 80, fixed in the SDK.
SENSOR_HTTP_PORT = 80

DEFAULT_FRAME_ID = "os_lidar"  # the vendor's lidar_frame and point_cloud_frame
DEFAULT_TOPIC = "points"  # the node's own topic name, relative: /<namespace>/points
PROC_MASK = "PCL|IMU|TLM"  # point clouds, IMU, telemetry; no images, no raw packets, no LaserScan

TRUE_WORDS = ("1", "true", "yes", "on")
FALSE_WORDS = ("0", "false", "no", "off")
# TIME_MODE -> the driver's timestamp_mode parameter.
# host: TIME_FROM_ROS_TIME, the driver stamps with the host's receive time of the first packet of
#       a scan and leaves the sensor's own timestamp_mode as it is.
# sensor/gps/ptp: the driver sets the sensor's timestamp_mode and stamps with the packets' time.
TIME_MODES = {
    "host": "TIME_FROM_ROS_TIME",
    "sensor": "TIME_FROM_INTERNAL_OSC",
    "gps": "TIME_FROM_SYNC_PULSE_IN",
    "ptp": "TIME_FROM_PTP_1588",
}
REFUSED_TIME_MODES = {
    "ntp": "Ouster sensors have no NTP client; use ptp, gps (sync pulse in), sensor or host",
}
DEFAULT_TIME_MODE = "host"

CONTRACT_SWITCH = "METROLLA_DRIVER_CONTRACT"
CONTRACT_VERSIONS = ("1",)


# --------------------------------------------------------------------------------------------
# Models. The driver reads the model from the sensor itself (sensor_info.prod_line); LIDAR_MODEL
# says which sensor the deployment expects. Ouster's README: OS0, OS1, OS2 and OSDome, firmware
# 2.0 or later.
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Model:
    key: str  # normalised name: lowercase letters and digits only
    label: str  # Ouster's name
    family: str
    channels: int | None  # None: any channel count of the family


_FAMILIES = (("os0", "OS0", (32, 64, 128)), ("os1", "OS1", (32, 64, 128)),
             ("os2", "OS2", (32, 64, 128)), ("osdome", "OSDome", (32, 64, 128)))


def normalise_model(value: str) -> str:
    """Lowercase, without an optional 'ouster-' prefix, '-' and '_' removed. Anything else (a
    space, a dot) is kept, so it cannot match a model and the name is refused."""
    key = value.lower()
    if key.startswith("ouster-") or key.startswith("ouster_"):
        key = key[len("ouster-"):]
    return key.replace("-", "").replace("_", "")


def _models() -> dict[str, Model]:
    table: dict[str, Model] = {}
    for key, label, channels in _FAMILIES:
        table[key] = Model(key, label, label, None)
        for n in channels:
            table[f"{key}{n}"] = Model(f"{key}{n}", f"{label}-{n}", label, n)
    return table


MODELS = _models()
# Ouster's product-line spelling (sensor_info.prod_line "OS-1-128", "OS-DOME-128") normalises to
# the same keys, so it is accepted as it is. A beam-configuration suffix ("OS-1-128-U0") is not.


def model_names() -> str:
    names = [m.label.lower() for m in MODELS.values() if m.channels is None]
    with_channels = [m.label.lower() for m in MODELS.values() if m.channels is not None]
    return ", ".join(names) + "; or with the channel count: " + ", ".join(with_channels)


def lookup_model(raw: str) -> Model | str:
    key = normalise_model(raw)
    if key in MODELS:
        return MODELS[key]
    return ("is not an Ouster model this image accepts; accepted (any case, optional 'ouster-' "
            f"prefix, '-' and '_' ignored, no beam-configuration suffix): {model_names()}")


# --------------------------------------------------------------------------------------------
# Environment variables.
# --------------------------------------------------------------------------------------------

REQUIRED = ("LIDAR_MODEL", "SENSOR_IP")
OPTIONAL = ("SENSOR_ID", "DATA_PORT", "DEVICE_PORT", "HOST_IP", "FRAME_ID", "POINTCLOUD_TOPIC",
            "TIME_MODE", "PUBLISH_SCAN", "OUSTER_RUN_DIR")
COMPAT = ("DUAL_LIDAR",)  # only false: one container per sensor
KNOWN = REQUIRED + OPTIONAL + COMPAT
# Names the fleets' .env files and the vendor's launch arguments use. In contract mode they are
# refused with the contract's name, so a compose that switched the contract on but kept an old
# name fails loudly instead of being ignored.
LEGACY = {
    "SENSOR_PORT": "DATA_PORT", "SENSOR_TYPE": "LIDAR_MODEL", "SENSOR_HOSTNAME": "SENSOR_IP",
    "LIDAR_PORT": "DATA_PORT", "IMU_PORT": None, "UDP_DEST": "HOST_IP", "LIDAR_IP": "SENSOR_IP",
    "TIMESTAMP_MODE": "TIME_MODE", "OUSTER_NS": "SENSOR_ID", "LIDAR_MODE": None,
    "UDP_PROFILE_LIDAR": None, "PROC_MASK": None, "PARAMS_FILE": None,
}
LEGACY_WHY = {
    "IMU_PORT": "the IMU port is DATA_PORT + 1 in this contract",
    "LIDAR_MODE": "the contract keeps the sensor's own lidar mode (resolution and rate)",
    "UDP_PROFILE_LIDAR": "the contract keeps the sensor's own packet profile",
    "PROC_MASK": f"the contract publishes {PROC_MASK}",
    "PARAMS_FILE": "the contract renders the parameter file itself",
}
ALIASES = {"SENSOR_MODEL": "LIDAR_MODEL", "LIDAR_TYPE": "LIDAR_MODEL", "FRAME_TOPIC": "POINTCLOUD_TOPIC",
           "TIME_SERVICE": "TIME_MODE", "HOST_ADDRESS": "HOST_IP", "SENSOR_HOST_IP": "HOST_IP",
           "SENSOR_FRAME_ID": "FRAME_ID", "SENSOR_POINTS_TOPIC": "POINTCLOUD_TOPIC",
           "POINT_CLOUD_TOPIC": "POINTCLOUD_TOPIC", "COMPUTER_IP": "HOST_IP"}
STRICT_PREFIXES = ("OUSTER_", "LIDAR_", "SENSOR_", "DATA_", "DEVICE_", "HOST_", "TIME_", "FRAME_",
                   "POINTCLOUD_", "POINT_", "PUBLISH_", "IMU_", "UDP_", "DUAL_", "TIMESTAMP_",
                   "PROC_", "COMPUTER_", "PARAMS_")

ACCEPTED = {
    "LIDAR_MODEL": "an Ouster model name such as os1-128, os0 or osdome-128 (the full list is in "
                   "the message when the name is unknown)",
    "SENSOR_IP": "the sensor's own IPv4 address, for example 192.168.1.201",
    "SENSOR_ID": "a lowercase letter then up to 30 lowercase letters, digits or single '_'",
    "DATA_PORT": f"a UDP port {MIN_PORT}-{MAX_DATA_PORT} (default {DEFAULT_DATA_PORT}; the IMU "
                 "port is DATA_PORT + 1)",
    "DEVICE_PORT": f"{SENSOR_HTTP_PORT} (the sensor's HTTP API port, fixed in the Ouster SDK)",
    "HOST_IP": "the host's IPv4 address on the sensor network, optionally with /1-/32",
    "FRAME_ID": "letters, digits and '_', not starting with a digit (default "
                f"{DEFAULT_FRAME_ID}, or <SENSOR_ID>_{DEFAULT_FRAME_ID} with SENSOR_ID)",
    "POINTCLOUD_TOPIC": f"a ROS topic name (default {DEFAULT_TOPIC})",
    "TIME_MODE": f"host, sensor, gps or ptp (default {DEFAULT_TIME_MODE})",
    "PUBLISH_SCAN": "0, false, no or off (the contract publishes no LaserScan)",
    "OUSTER_RUN_DIR": f"an absolute directory path (default {DEFAULT_RUN_DIR})",
}

_PORT_RE = re.compile(r"[1-9][0-9]{0,4}")
_OCTET = r"(?:25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])"
_IPV4_RE = re.compile(rf"{_OCTET}\.{_OCTET}\.{_OCTET}\.{_OCTET}")
_PREFIX_RE = re.compile(r"[1-9]|[12][0-9]|3[0-2]")
_SENSOR_ID_RE = re.compile(r"[a-z][a-z0-9_]{0,30}")  # metrolla-edge deployment schema, sensors[].id
_FRAME_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
_TOPIC_RE = re.compile(r"/?[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*")
_RUN_DIR_RE = re.compile(r"/[A-Za-z0-9_./-]*")


@dataclass
class Settings:
    model: Model
    sensor_ip: str
    sensor_id: str | None
    data_port: int
    host_ip: str | None  # as given, maybe with /prefix
    frame_id: str
    topic: str
    time_mode: str
    run_dir: str
    sources: dict[str, str] = field(default_factory=dict)

    @property
    def imu_port(self) -> int:
        return self.data_port + 1

    @property
    def namespace(self) -> str:
        return "/" + (self.sensor_id or DEFAULT_NAMESPACE)

    @property
    def node_fqn(self) -> str:
        return self.namespace + "/" + NODE_NAME

    @property
    def topic_fqn(self) -> str:
        return self.topic if self.topic.startswith("/") else self.namespace + "/" + self.topic

    @property
    def sensor_frame(self) -> str:
        return f"{self.sensor_id}_os_sensor" if self.sensor_id else "os_sensor"

    @property
    def imu_frame(self) -> str:
        return f"{self.sensor_id}_os_imu" if self.sensor_id else "os_imu"

    @property
    def params_path(self) -> str:
        return self.run_dir.rstrip("/") + "/driver_params.yaml"

    @property
    def metadata_path(self) -> str:
        return self.run_dir.rstrip("/") + "/metadata.json"


class Refused(Exception):
    def __init__(self, problems: list[str]):
        super().__init__("\n".join(problems))
        self.problems = problems


class BrokenImage(Exception):
    """The image's own files do not have the shape this script expects (a build problem)."""


def shown(value: str) -> str:
    text = value if len(value) <= 64 else value[:61] + "..."
    if text.isprintable() and text.isascii():
        return "'" + text + "'"
    return ascii(text)


def _ipv4(name: str, value: str, problems: list[str], what: str) -> ipaddress.IPv4Address | None:
    if not _IPV4_RE.fullmatch(value):
        problems.append(f"{name}={shown(value)} is not an IPv4 address; accepted: {what} in dotted "
                        "decimal without leading zeros, for example 192.168.1.201")
        return None
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_reserved or \
            address == ipaddress.IPv4Address("255.255.255.255"):
        problems.append(f"{name}={shown(value)} is not a unicast address; accepted: {what}, not "
                        "0.0.0.0, multicast (224-239.x.x.x), 240.x.x.x and above or broadcast")
        return None
    return address


def _bool(name: str, value: str, problems: list[str], default_text: str) -> bool | None:
    word = value.lower()
    if word in TRUE_WORDS:
        return True
    if word in FALSE_WORDS:
        return False
    problems.append(f"{name}={shown(value)} is not a boolean; accepted: {', '.join(TRUE_WORDS)} or "
                    f"{', '.join(FALSE_WORDS)}, in any case ({default_text})")
    return None


def parse(env: Mapping[str, str]) -> Settings:
    """Validate the environment. Raises Refused with one line per problem."""
    problems: list[str] = []
    sources: dict[str, str] = {}

    def get(name: str) -> str | None:
        value = env.get(name)
        sources[name] = "default" if value is None else "env"
        return value

    switch = env.get(CONTRACT_SWITCH)
    if switch not in CONTRACT_VERSIONS:
        problems.append(f"{CONTRACT_SWITCH}={shown(switch or '')} is not a contract version this "
                        f"image knows; accepted: {', '.join(CONTRACT_VERSIONS)} (or unset or 0 for "
                        "the legacy entrypoint)")

    for name in REQUIRED + OPTIONAL:  # an empty DUAL_LIDAR means false, as in the other images
        if name in env and env[name] == "":
            problems.append(f"{name} is set but empty; accepted: {ACCEPTED[name]}"
                            + ("" if name in REQUIRED else ", or unset it for the default"))
    present = {k: v for k, v in env.items() if k in KNOWN and v != ""}

    # LIDAR_MODEL
    model = None
    if "LIDAR_MODEL" not in env:
        problems.append("LIDAR_MODEL is not set; accepted (any case, optional 'ouster-' prefix, "
                        f"'-' and '_' ignored): {model_names()}")
    elif "LIDAR_MODEL" in present:
        found = lookup_model(present["LIDAR_MODEL"])
        if isinstance(found, str):
            problems.append(f"LIDAR_MODEL={shown(present['LIDAR_MODEL'])} {found}")
        else:
            model = found
    get("LIDAR_MODEL")

    # SENSOR_IP
    sensor = None
    if "SENSOR_IP" not in env:
        problems.append("SENSOR_IP is not set; accepted: the sensor's own IPv4 address, for example "
                        "192.168.1.201 (the driver configures the sensor over HTTP at that "
                        "address and reads its calibration from it)")
    elif "SENSOR_IP" in present:
        sensor = _ipv4("SENSOR_IP", present["SENSOR_IP"], problems, "the sensor's own IPv4 address")
    get("SENSOR_IP")

    # SENSOR_ID
    sensor_id = None
    value = get("SENSOR_ID")
    if value:
        if not _SENSOR_ID_RE.fullmatch(value) or "__" in value:
            problems.append(f"SENSOR_ID={shown(value)} is not a sensor id; accepted: 1-31 characters, "
                            "a lowercase letter then lowercase letters, digits or single '_' (it "
                            "becomes the node's ROS namespace)")
        else:
            sensor_id = value

    # DATA_PORT
    data_port = DEFAULT_DATA_PORT
    value = get("DATA_PORT")
    if value:
        if not _PORT_RE.fullmatch(value) or not MIN_PORT <= int(value) <= MAX_DATA_PORT:
            problems.append(f"DATA_PORT={shown(value)} is not a usable port; accepted: a whole "
                            f"number {MIN_PORT}-{MAX_DATA_PORT} without leading zeros (the UDP port "
                            f"the sensor sends lidar data to, default {DEFAULT_DATA_PORT}; IMU data "
                            "goes to DATA_PORT + 1)")
        else:
            data_port = int(value)

    # DEVICE_PORT: only the fixed HTTP port is accepted, so a uniform generator can pass it.
    value = get("DEVICE_PORT")
    if value and value != str(SENSOR_HTTP_PORT):
        problems.append(f"DEVICE_PORT={shown(value)} cannot be changed for Ouster sensors; accepted: "
                        f"{SENSOR_HTTP_PORT} or unset (the driver configures the sensor through its "
                        f"HTTP API, http://SENSOR_IP/api/v1/..., port {SENSOR_HTTP_PORT} fixed in "
                        "the Ouster SDK)")

    # HOST_IP
    host_ip = None
    value = get("HOST_IP")
    if value:
        address, _, prefix = value.partition("/")
        if "/" in value and not _PREFIX_RE.fullmatch(prefix):
            problems.append(f"HOST_IP={shown(value)} has no usable prefix length; accepted: the "
                            "host's IPv4 address on the sensor network, optionally with /1-/32, for "
                            "example 192.168.1.100/24")
        else:
            host = _ipv4("HOST_IP", address, problems,
                         "the host's IPv4 address on the sensor network (optionally with /1-/32)")
            if host is not None:
                host_ip = value
                if sensor is not None and host == sensor:
                    problems.append(f"HOST_IP={shown(value)} is the sensor's own address; accepted: "
                                    "the host's address on the sensor network")
                    host_ip = None
                elif sensor is not None and prefix:
                    network = ipaddress.IPv4Interface(value).network
                    if sensor not in network:
                        problems.append(f"HOST_IP={shown(value)}: the sensor at {sensor} is not in "
                                        f"the host's subnet {network}; accepted: a host address in "
                                        "the sensor's subnet")
                        host_ip = None

    # FRAME_ID. Without SENSOR_ID the vendor's frame names; with it, every frame this node
    # publishes is prefixed with the id, so two sensors never publish the same TF frame.
    frame_id = f"{sensor_id}_{DEFAULT_FRAME_ID}" if sensor_id else DEFAULT_FRAME_ID
    value = get("FRAME_ID")
    if value:
        if not _FRAME_RE.fullmatch(value):
            problems.append(f"FRAME_ID={shown(value)} is not a frame id; accepted: 1-64 characters, "
                            "letters, digits and '_', not starting with a digit (default "
                            f"{DEFAULT_FRAME_ID}, or <SENSOR_ID>_{DEFAULT_FRAME_ID} with SENSOR_ID)")
        else:
            frame_id = value
            derived = {f"{sensor_id}_os_sensor" if sensor_id else "os_sensor",
                       f"{sensor_id}_os_imu" if sensor_id else "os_imu"}
            if value in derived:
                problems.append(f"FRAME_ID={shown(value)} is the name of the sensor's housing or IMU "
                                f"frame in this image ({', '.join(sorted(derived))}); accepted: "
                                "another name")

    # POINTCLOUD_TOPIC
    topic = DEFAULT_TOPIC
    value = get("POINTCLOUD_TOPIC")
    if value:
        if not _TOPIC_RE.fullmatch(value) or "__" in value or len(value) > 200:
            problems.append(f"POINTCLOUD_TOPIC={shown(value)} is not a ROS topic name; accepted: "
                            "'/'-separated tokens of letters, digits and single '_', each starting "
                            "with a letter, absolute (/x) or relative to the node's namespace "
                            f"(default {DEFAULT_TOPIC})")
        else:
            topic = value

    # TIME_MODE
    time_mode = DEFAULT_TIME_MODE
    value = get("TIME_MODE")
    if value:
        word = value.lower()
        if word in REFUSED_TIME_MODES:
            problems.append(f"TIME_MODE={shown(value)} is not available: "
                            f"{REFUSED_TIME_MODES[word]}")
        elif word not in TIME_MODES:
            problems.append(f"TIME_MODE={shown(value)} is not a time mode; accepted, any case: host "
                            "(the default: stamp with the host's receive time, the sensor's own "
                            "time source is left as it is), sensor (the sensor's internal clock), "
                            "gps (sync pulse in) or ptp (IEEE 1588); sensor, gps and ptp set the "
                            "sensor's timestamp_mode")
        else:
            time_mode = word

    # PUBLISH_SCAN
    value = get("PUBLISH_SCAN")
    if value:
        flag = _bool("PUBLISH_SCAN", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"PUBLISH_SCAN={shown(value)} is not supported with {CONTRACT_SWITCH}; "
                            "accepted: unset or false. The contract publishes PointCloud2 and IMU "
                            "only")

    # OUSTER_RUN_DIR
    run_dir = DEFAULT_RUN_DIR
    value = get("OUSTER_RUN_DIR")
    if value:
        if not _RUN_DIR_RE.fullmatch(value) or "/../" in value + "/":
            problems.append(f"OUSTER_RUN_DIR={shown(value)} is not an absolute directory path; "
                            "accepted: an absolute path of letters, digits, '_', '.', '-' and '/' "
                            f"(default {DEFAULT_RUN_DIR})")
        else:
            run_dir = value

    # DUAL_LIDAR
    value = env.get("DUAL_LIDAR")
    if value:
        flag = _bool("DUAL_LIDAR", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"DUAL_LIDAR={shown(value)} is not supported with {CONTRACT_SWITCH}; "
                            "accepted: unset or false. Run one container per Ouster sensor")

    # Unknown variables under this image's prefixes, and the legacy names.
    for name in sorted(env):
        if name in KNOWN:
            continue
        if name in LEGACY:
            instead = LEGACY[name]
            what = f"use {instead}" if instead else LEGACY_WHY[name] + "; remove it"
            problems.append(f"{name} is a legacy name and is not read with {CONTRACT_SWITCH}=1; "
                            f"{what}")
            continue
        if not name.startswith(STRICT_PREFIXES):
            continue
        close = [ALIASES[name]] if name in ALIASES else difflib.get_close_matches(name, KNOWN, n=1,
                                                                                    cutoff=0.6)
        hint = f" (did you mean {close[0]}?)" if close else ""
        problems.append(f"{name} is not a variable this image knows{hint}; accepted: "
                        f"{', '.join(KNOWN)}")

    if problems:
        raise Refused(problems)
    assert model is not None and sensor is not None
    return Settings(model=model, sensor_ip=str(sensor), sensor_id=sensor_id, data_port=data_port,
                    host_ip=host_ip, frame_id=frame_id, topic=topic, time_mode=time_mode,
                    run_dir=run_dir, sources=sources)


# --------------------------------------------------------------------------------------------
# Rendering: the vendor's driver_params.yaml, line by line, with the contract's keys replaced.
# Each replaced key must be found exactly once, or the image is broken (selfcheck fails the
# build), so a vendor update that renames a parameter cannot slip through.
# --------------------------------------------------------------------------------------------

VENDOR_NODE_KEY = "ouster/os_driver:"
_PARAM_RE = re.compile(r"^(?P<indent>    )(?P<key>[a-z_]+):(?P<rest>.*)$")


def yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    if isinstance(value, str):
        return json.dumps(value)  # a JSON string is a YAML double-quoted scalar
    raise TypeError(f"cannot render {value!r}")


def contract_params(s: Settings) -> dict[str, object]:
    host = s.host_ip.partition("/")[0] if s.host_ip else ""
    return {
        "sensor_hostname": s.sensor_ip,
        "udp_dest": host,  # empty: the sensor sends to the address the driver contacts it from
        "mtp_dest": "",
        "mtp_main": False,
        "timestamp_mode": TIME_MODES[s.time_mode],
        "metadata": s.metadata_path,
        "lidar_port": s.data_port,
        "imu_port": s.imu_port,
        "sensor_frame": s.sensor_frame,
        "lidar_frame": s.frame_id,
        "imu_frame": s.imu_frame,
        "point_cloud_frame": s.frame_id,
        "pub_static_tf": True,
        "proc_mask": PROC_MASK,
        "persist_config": False,
        # Keep trying while the sensor is unreachable (boot order, power loss) instead of sitting
        # unconfigured; the retries are logged.
        "attempt_reconnect": True,
        "dormant_period_between_reconnects": 5.0,
    }


def render_params(s: Settings, source: str) -> str:
    lines = source.split("\n")
    wanted = contract_params(s)
    seen: dict[str, int] = {}
    node_keys = 0
    for i, line in enumerate(lines):
        if line.rstrip() == VENDOR_NODE_KEY:
            node_keys += 1
            lines[i] = "/**:"
            continue
        m = _PARAM_RE.match(line)
        if m and m["key"] in wanted:
            seen[m["key"]] = seen.get(m["key"], 0) + 1
            lines[i] = f"{m['indent']}{m['key']}: {yaml_scalar(wanted[m['key']])}"
        elif m and m["key"] == "auto_start":
            raise BrokenImage("driver_params.yaml already sets auto_start")
    if node_keys != 1:
        raise BrokenImage(f"driver_params.yaml: expected '{VENDOR_NODE_KEY}' exactly once")
    missing = [k for k in wanted if seen.get(k, 0) != 1]
    if missing:
        raise BrokenImage("driver_params.yaml: expected each of these exactly once: "
                          + ", ".join(missing))
    body = "\n".join(lines).rstrip("\n")
    header = [
        f"# Rendered from the environment by the {PROG} image ({CONTRACT_SWITCH}=1); do not edit.",
        f"# LIDAR_MODEL {s.model.label}: the driver reads the model and calibration from the "
        "sensor; this is the expected model.",
        f"# TIME_MODE {s.time_mode}: timestamp_mode {TIME_MODES[s.time_mode]}"
        + (" (host receive time; the sensor's time source is not changed)" if s.time_mode == "host"
           else " (set on the sensor; points stamped with the sensor's time)"),
        f"# Ports: lidar {s.data_port}, IMU {s.imu_port} (UDP, host side).",
    ]
    footer = [
        "    # Added by the contract: the node starts itself (configure, activate), no launch file.",
        "    auto_start: true",
    ]
    return "\n".join(header) + "\n" + body + "\n" + "\n".join(footer) + "\n"


def read_vendor_params(path: str | None = None) -> str:
    with open(path or VENDOR_PARAMS, encoding="utf-8") as handle:
        return handle.read()


def render(s: Settings, params_path: str | None = None) -> str:
    return render_params(s, read_vendor_params(params_path))


def node_argv(s: Settings) -> list[str]:
    argv = [DRIVER_NODE, "--ros-args", "-r", f"__ns:={s.namespace}", "-r", f"__node:={NODE_NAME}",
            "--params-file", s.params_path]
    if s.topic != DEFAULT_TOPIC:
        argv += ["-r", f"points:={s.topic}"]
    return argv


def describe(s: Settings) -> dict:
    return {
        "model": s.model.label,
        "sensor_ip": s.sensor_ip,
        "data_port": s.data_port,
        "imu_port": s.imu_port,
        "sensor_http_port": SENSOR_HTTP_PORT,
        "host_ip": s.host_ip,
        "udp_dest": contract_params(s)["udp_dest"],
        "frame_id": s.frame_id,
        "sensor_frame": s.sensor_frame,
        "imu_frame": s.imu_frame,
        "pointcloud_topic": s.topic_fqn,
        "time_mode": s.time_mode,
        "timestamp_mode": TIME_MODES[s.time_mode],
        "node": s.node_fqn,
        "params_path": s.params_path,
        "metadata_path": s.metadata_path,
        "argv": node_argv(s),
        "sources": dict(sorted(s.sources.items())),
    }


# --------------------------------------------------------------------------------------------
# Command line.
# --------------------------------------------------------------------------------------------


def refuse(problems: list[str], as_json: bool = False) -> int:
    if as_json:
        print(json.dumps({"refused": problems}, indent=2))
    for problem in problems:
        print(f"{PROG}: refusing to start: {problem}", file=sys.stderr)
    return EXIT_REFUSED


def broken(message: str) -> int:
    print(f"{PROG}: broken image: {message}", file=sys.stderr)
    return EXIT_REFUSED


def write_params(s: Settings, text: str) -> None:
    os.makedirs(s.run_dir, mode=0o700, exist_ok=True)
    tmp = s.params_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, s.params_path)


def exec_env(env: Mapping[str, str]) -> dict[str, str]:
    out = dict(env)
    out.setdefault("ROS_HOME", DEFAULT_ROS_HOME)
    out.setdefault("ROS_LOG_DIR", out["ROS_HOME"].rstrip("/") + "/log")
    return out


def cmd_run(env: Mapping[str, str]) -> int:
    try:
        s = parse(env)
        text = render(s)
    except Refused as exc:
        return refuse(exc.problems)
    except (BrokenImage, OSError) as exc:
        return broken(str(exc))
    child_env = exec_env(env)
    try:
        write_params(s, text)
        os.makedirs(child_env["ROS_LOG_DIR"], exist_ok=True)
    except OSError as exc:
        return refuse([f"cannot write {exc.filename or s.run_dir} ({exc.strerror}) as uid "
                       f"{os.getuid()}; accepted: a writable /tmp, for example docker run "
                       "--tmpfs /tmp or compose 'tmpfs: [/tmp]' with a read-only root filesystem "
                       "(or point OUSTER_RUN_DIR and ROS_LOG_DIR at a writable volume)"])
    print(f"{PROG}: Ouster {s.model.label} at {s.sensor_ip}, lidar port {s.data_port}, IMU port "
          f"{s.imu_port}, node {s.node_fqn}, topic {s.topic_fqn}, frame {s.frame_id}, time "
          f"{s.time_mode}, parameters {s.params_path}", file=sys.stderr, flush=True)
    argv = node_argv(s)
    try:
        os.execve(argv[0], argv, child_env)
    except OSError as exc:
        print(f"{PROG}: cannot start the driver {argv[0]}: {exc.strerror} (a broken image)",
              file=sys.stderr)
    return EXIT_REFUSED


def cmd_check(env: Mapping[str, str], as_json: bool) -> int:
    try:
        s = parse(env)
        text = render(s)
    except Refused as exc:
        return refuse(exc.problems, as_json)
    except (BrokenImage, OSError) as exc:
        return broken(str(exc))
    if as_json:
        out = describe(s)
        out["params_yaml"] = text
        print(json.dumps(out, indent=2))
    else:
        sys.stdout.write(text)
        print("# exec: " + " ".join(node_argv(s)))
    return EXIT_OK


def cmd_selfcheck(params_path: str | None = None, node: str | None = None) -> int:
    """Build-time check: every model renders against the vendor file, the node exists."""
    problems = []
    for model in MODELS.values():
        env = {CONTRACT_SWITCH: "1", "LIDAR_MODEL": model.label, "SENSOR_IP": "192.168.1.201"}
        try:
            render(parse(env), params_path)
        except (Refused, BrokenImage, OSError) as exc:
            problems.append(f"{model.label}: {exc}")
    node = node or DRIVER_NODE
    if not os.access(node, os.X_OK):
        problems.append(f"driver binary missing or not executable: {node}")
    for p in problems:
        print(f"{PROG}: selfcheck FAILED: {p}", file=sys.stderr)
    if not problems:
        print(f"{PROG}: selfcheck ok: {len(MODELS)} model names render, {node} present")
    return EXIT_REFUSED if problems else EXIT_OK


# The image's default CMD (legacy). With the contract switched on it means "run".
LEGACY_DEFAULT_CMD = ["ros2", "launch", "ouster_ros", "driver.launch.py",
                      "sensor_hostname:=192.168.1.201", "viz:=false"]
USAGE = f"usage: {PROG} [run | check [--json] | selfcheck]  (with {CONTRACT_SWITCH}=1; see docs/METROLLA_DRIVER_CONTRACT.md)"


def main(argv: list[str], env: Mapping[str, str]) -> int:
    if not argv or argv == ["run"] or argv == LEGACY_DEFAULT_CMD:
        return cmd_run(env)
    if argv[0] == "check" and argv[1:] in ([], ["--json"]):
        return cmd_check(env, argv[1:] == ["--json"])
    if argv == ["selfcheck"]:
        return cmd_selfcheck()
    if argv[0] in ("-h", "--help", "help"):
        print(USAGE)
        return EXIT_OK
    print(f"{PROG}: refusing to start: unknown command {shown(' '.join(argv))} with "
          f"{CONTRACT_SWITCH}=1; accepted: run (the default), check, check --json. The contract "
          "renders the parameters from the environment; to run something else, unset "
          f"{CONTRACT_SWITCH} or override the entrypoint", file=sys.stderr)
    return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:], os.environ))

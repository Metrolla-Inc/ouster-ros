#!/usr/bin/python3 -I
"""sensor-tool for Ouster lidars: reads a sensor's own settings and sets them. Interface version 1.

    sensor-tool probe    --ip A [--model M] [--timeout S] [--driver-stopped]
    sensor-tool get-net  --ip A [--model M] [--timeout S] [--driver-stopped]
    sensor-tool set-net  --ip A --new-ip B --netmask M --gateway G [--no-reboot] [--dry-run] [--verify-timeout S] --driver-stopped
    sensor-tool set-dest --ip A --dest-ip H --data-port P [--dry-run] [--verify-timeout S] --driver-stopped
    sensor-tool set-ptp  --ip A --transport L2|UDPv4 --domain N --delay-mechanism E2E|P2P --timescale UTC|TAI
                         [--profile 1588v2] [--clock-source ptp] [--dry-run] [--verify-timeout S] --driver-stopped

It prints exactly one JSON object, also for a usage error, and exits 0 (reported, set, unchanged),
2 (changes: a dry run that would change the sensor), 1 (refused, failed, no-answer), 3 (unsupported)
or 5 (unverified: a change was sent and did not read back in time). Standard library only.

An Ouster's whole interface is HTTP on TCP port 80. Reads and dry runs work beside the running
driver and need no `--driver-stopped`; every write needs it.

The rules it keeps (tests/unit/test_sensor_tool.py has a test for each):

- Reading and writing are separate code. On this sensor a GET can be a write (the SDK's
  `set_config_param`, `reinitialize` and `save_config_params` are GETs), so the line is drawn by
  path: `read` sends a GET to one of the four paths of READS and nothing else; the paths of
  WRITES go through `Writer`, which exists only in a run that was told `--driver-stopped` and is
  not a dry run. No other path can be sent at all (no DELETE of the address, no
  `set_udp_dest_auto`, no firmware or restart call).
- Every write needs `--driver-stopped`.
- A set command reads first, changes only what differs, reads back, and is idempotent.
- One connection at a time, counted in `sessions`.
- What cannot be set is said in `unsupported`, never skipped silently.

Where each fact comes from. "SDK" is the Ouster SDK this repository builds, the submodule
ouster-ros/ouster-sdk at commit 483206f, directory ouster_client/src:

- `GET api/v1/sensor/metadata/sensor_info`: SDK sensor_http_imp.cpp:18-20. Its keys `prod_line`,
  `prod_sn`, `image_rev`: SDK sensor_info.cpp:254-259.
- `GET api/v1/sensor/cmd/get_config_param?args=active`: SDK sensor_http_imp.cpp:36-38.
- The configuration keys `udp_dest` (and the old `udp_ip`), `udp_port_lidar`, `udp_port_imu`,
  `timestamp_mode`: SDK types.cpp:612-628 (read) and 794-804 (written). The values of
  `timestamp_mode`, among them `TIME_FROM_PTP_1588`: SDK types.cpp:50-54.
- A configuration change is made as the SDK's set_config makes it (client.cpp:296-371): the
  active parameters with the changed keys are staged whole (client.cpp:351-359) with
  `set_config_param?args=.+<JSON>`, answered `"set_config_param"` (sensor_http_imp.cpp:28-34), so
  that nothing staged earlier comes along; then `reinitialize`, answered `{}`
  (sensor_http_imp.cpp:69-71; client.cpp:361); then `save_config_params`, answered `{}`
  (sensor_http_imp.cpp:73-75; client.cpp:365-367), which keeps them over a power cycle. `udp_ip`
  is written beside `udp_dest` where the sensor has it (client.cpp:341-343).
- The IMU port is the data port plus one: this repository's driver contract, docker/ouster_env.py
  ("the IMU port is DATA_PORT + 1").

**Not from the SDK, which has no call for any of it: the address and the PTP profile.** These are
from Ouster's firmware user manual (HTTP API v1), as recalled; the manual is not in this
repository and **a real sensor must confirm every one of them**:

- `GET api/v1/system/network`: `ethaddr`, and `ipv4` with `addr` ("<address>/<prefix>"),
  `link_local` and `override` (null, or the static address).
- `PUT api/v1/system/network/ipv4/override` with a JSON string as the body. The sensor resets its
  network within about a second and then answers only at the new address. This tool sends
  `"<address>/<prefix>"`: whether the sensor accepts the form with the prefix is not confirmed.
  A sensor that refuses the body answers an error, and the run ends `failed` with nothing changed.
  DELETE of the override returns the sensor to DHCP; this tool never sends it.
- `GET api/v1/time/ptp` (`profile`, and `port_data_set.port_state`), and
  `PUT api/v1/time/ptp/profile` with a JSON string: `default`, `gptp`, `automotive-slave`,
  `default-l2-relaxed`. What each profile means (PROFILES below: transport, delay mechanism,
  domain 0) is this tool's reading of the manual and of the profiles' names; `default-l2-relaxed`
  as "L2 with end-to-end delay" is read from its name alone.

Not possible, and said in the answer (`unsupported`) when it is asked for:

- **the gateway**: no call reads or sets one (asked as anything but 0.0.0.0);
- **a PTP domain other than 0**, and **peer-to-peer delay over UDPv4**: no firmware profile has them;
- **PTP transport and delay mechanism on a sensor that does not show its profile**: what cannot
  be read is not written;
- **the address of a sensor that does not show its network settings**: the same.

A restart is not needed and there is no call for one here: `--no-reboot` changes nothing. The
timescale is the grandmaster's and is not a sensor setting. None of this has been run against a
real sensor.
"""

from __future__ import annotations

import argparse
import http.client
import ipaddress
import json
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

VENDOR = "ouster"

# ---- the interface: this part is the same text in every driver repository's sensor-tool ----------

INTERFACE = 1
COMMANDS = ("probe", "get-net", "set-net", "set-dest", "set-ptp")
STEP = {"set-net": "net", "set-dest": "dest", "set-ptp": "ptp"}
EXIT = {"reported": 0, "set": 0, "unchanged": 0, "changes": 2, "refused": 1, "failed": 1, "no-answer": 1, "unsupported": 3, "unverified": 5}
PAUSE = 1.0  # seconds between two reads while a change is awaited; tests shorten it


class Stop(Exception):
    """The run ends with this outcome and message."""

    def __init__(self, outcome: str, message: str) -> None:
        super().__init__(message)
        self.outcome, self.message = outcome, message


class NoAnswer(Exception):
    """Nothing answered, or what answered is not this vendor's sensor."""


class Sessions:
    """Counts the connections of a run. A second control connection while one is open is a fault
    of this program, and is refused here before it is opened."""

    def __init__(self, control: tuple, others: tuple = ()) -> None:
        self.control, self.open_now, self.max_concurrent = control, 0, 0
        self.counts = {kind: 0 for kind in (*control, *others)}

    def opening(self, kind: str) -> None:
        if kind in self.control:
            if self.open_now:
                raise RuntimeError(f"a second control connection ({kind}) while one is open")
            self.open_now += 1
            self.max_concurrent = max(self.max_concurrent, self.open_now)
        self.counts[kind] += 1

    def not_opened(self, kind: str) -> None:
        """The connection just announced was never made."""
        self.counts[kind] -= 1
        self.closed(kind)

    def closed(self, kind: str) -> None:
        if kind in self.control:
            self.open_now -= 1

    def report(self) -> dict:
        return {**self.counts, "max_concurrent": self.max_concurrent}


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:  # argparse would print usage and exit 2
        raise Stop("refused", f"usage: {message}")


def ipv4(text: str) -> str:
    try:
        return str(ipaddress.IPv4Address(text))
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r} is not an IPv4 address") from None


def netmask(text: str) -> str:
    text = ipv4(text)
    bits = int(ipaddress.IPv4Address(text))
    if bits == 0 or (~bits & 0xFFFFFFFF) & ((~bits & 0xFFFFFFFF) + 1):
        raise argparse.ArgumentTypeError(f"{text} is not a netmask")
    return text


def number(low: int, high: int):
    def parse(text: str) -> int:
        if not text.isascii() or not text.isdigit() or not low <= int(text) <= high:
            raise argparse.ArgumentTypeError(f"{text!r} is not a number from {low} to {high}")
        return int(text)
    return parse


def seconds(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        value = -1.0
    if not 0 <= value <= 3600:
        raise argparse.ArgumentTypeError(f"{text!r} is not a number of seconds from 0 to 3600")
    return value


def parse_arguments(argv: list, extra=None) -> argparse.Namespace:
    """The arguments of interface version 1. `extra(parser, command)` adds a vendor's own options."""
    parser = _Parser(prog="sensor-tool", add_help=False, allow_abbrev=False)
    commands = parser.add_subparsers(dest="command", parser_class=_Parser, required=True)
    for name in COMMANDS:
        sub = commands.add_parser(name, add_help=False, allow_abbrev=False)
        sub.add_argument("--ip", type=ipv4, required=True)
        sub.add_argument("--model", default="")
        sub.add_argument("--dev-port", type=number(1, 65535), default=None)
        sub.add_argument("--timeout", type=seconds, default=5.0)
        sub.add_argument("--driver-stopped", action="store_true")
        if name == "set-net":
            sub.add_argument("--new-ip", type=ipv4, required=True)
            sub.add_argument("--netmask", type=netmask, required=True)
            sub.add_argument("--gateway", type=ipv4, required=True)
            sub.add_argument("--no-reboot", action="store_true")
        elif name == "set-dest":
            sub.add_argument("--dest-ip", type=ipv4, required=True)
            sub.add_argument("--data-port", type=number(1, 65535), required=True)
        elif name == "set-ptp":
            sub.add_argument("--transport", choices=("L2", "UDPv4"), required=True)
            sub.add_argument("--domain", type=number(0, 127), required=True)
            sub.add_argument("--delay-mechanism", choices=("E2E", "P2P"), required=True)
            sub.add_argument("--timescale", choices=("UTC", "TAI"), required=True)
            sub.add_argument("--profile", choices=("1588v2",), default="1588v2")
            sub.add_argument("--clock-source", choices=("ptp",), default="ptp")
        if name in STEP:
            sub.add_argument("--dry-run", action="store_true")
            sub.add_argument("--verify-timeout", type=seconds, default=60.0)
        if extra is not None:
            extra(sub, name)
    args = parser.parse_args(argv)
    args.dry_run = bool(getattr(args, "dry_run", False))
    args.writes = args.command in STEP and not args.dry_run
    return args


def blank(vendor: str) -> dict:
    return {"sensor_tool": INTERFACE, "vendor": vendor, "command": None, "ip": None, "outcome": None, "exit_code": None,
            "message": "", "identity": None, "before": None, "wanted": None, "after": None, "changed": [], "unsupported": [],
            "reboot": "none", "verified": None, "answers_at": None, "sessions": None, "warnings": []}


def blank_state() -> dict:
    return {"net": {"ip": None, "netmask": None, "gateway": None}, "dest": {"ip": None, "data_port": None},
            "ptp": {"clock_source": None, "profile": None, "transport": None, "domain": None, "status": None}, "channel": None}


def blank_identity(vendor: str) -> dict:
    return {"vendor": vendor, "model": None, "serial": None, "mac": None, "firmware": None}


def source_address(ip: str):
    """The address of this host that a packet to `ip` would leave from, or None. Sends nothing."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((ip, 9))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


def check_new_address(new_ip: str, mask: str, gateway: str) -> None:
    """Refuse an address that is no host address of its network, a gateway outside it, and an address
    this host could not reach afterwards: it has no address of its own in that network."""
    network = ipaddress.IPv4Network(f"{new_ip}/{mask}", strict=False)
    address = ipaddress.IPv4Address(new_ip)
    if address.is_multicast or address.is_unspecified or address.is_loopback and not ALLOW_LOOPBACK:
        raise Stop("refused", f"{new_ip} is not an address a sensor can have")
    if network.prefixlen < 31 and address in (network.network_address, network.broadcast_address):
        raise Stop("refused", f"{new_ip} is the network or broadcast address of {network}")
    if gateway != "0.0.0.0" and (ipaddress.IPv4Address(gateway) not in network or gateway == new_ip):
        raise Stop("refused", f"the gateway {gateway} is not another address of {network}")
    source = source_address(new_ip)
    if source is None or ipaddress.IPv4Address(source) not in network:
        raise Stop("refused", f"this host has no address in {network}, so the sensor could not be reached at {new_ip} afterwards; nothing was sent")
    if source == new_ip and not ALLOW_LOOPBACK:
        raise Stop("refused", f"{new_ip} is this host's own address")


ALLOW_LOOPBACK = False  # tests set it: their fake sensors are on loopback addresses


def differing(step: str, wanted: dict, have: dict) -> list:
    """The settings of `wanted` that `have` does not already have, as "step.key"."""
    return [f"{step}.{key}" for key, value in wanted.items() if (have or {}).get(key) != value]


def await_state(deadline: float, read, good):
    """Read until `good(state)` or the deadline: (the last state read or None, whether it was good).
    `read` returns None when nothing answered."""
    last = None
    while True:
        state = read()
        if state is not None:
            last = state
            if good(state):
                return state, True
        if time.monotonic() + PAUSE > deadline:
            return last, False
        time.sleep(PAUSE)


class Run:
    """One run of the tool: its arguments, its answer and its connections."""

    def __init__(self, args: argparse.Namespace, out: dict, sessions: Sessions) -> None:
        self.args, self.out, self.sessions = args, out, sessions

    def warn(self, text: str) -> None:
        if text not in self.out["warnings"]:
            self.out["warnings"].append(text)

    def cannot(self, setting: str, reason: str) -> None:
        self.out["unsupported"].append({"setting": setting, "reason": reason})

    def need_stopped(self, what: str) -> None:
        if not self.args.driver_stopped:
            raise Stop("refused", f"{what} needs --driver-stopped: the driver's own connection to the sensor must be closed first; nothing was sent")


def public(state):
    """`state` without the tool's own notes (keys that start with '_')."""
    if state is None:
        return None
    return {key: ({k: v for k, v in value.items() if not k.startswith("_")} if isinstance(value, dict) else value)
            for key, value in state.items()}


def settle(run: Run, step: str, wanted: dict, state: dict) -> list:
    """Note what is wanted and what differs. Ends the run when nothing differs, or on a dry run."""
    out = run.out
    out["before"], out["wanted"] = public(state), {step: wanted}
    out["changed"] = differing(step, wanted, state[step])
    if not out["changed"]:
        out["after"], out["verified"], out["answers_at"] = public(state), True, run.args.ip
        raise Stop("unchanged", f"{step} is already as asked; nothing was sent")
    if run.args.dry_run:
        raise Stop("changes", f"would change {', '.join(out['changed'])}")
    return out["changed"]


def finish(run: Run, step: str, wanted: dict, state, good: bool, at) -> None:
    out = run.out
    out["after"], out["verified"], out["answers_at"] = public(state), good, at
    if good:
        raise Stop("set", f"{', '.join(out['changed'])} set and read back")
    still = differing(step, wanted, state[step]) if state else out["changed"]
    raise Stop("unverified", f"the change was sent, and the sensor did not answer with the new values within "
               f"{run.args.verify_timeout:g} s (still different: {', '.join(still)})")


def run_tool(argv, vendor: str, sessions: Sessions, handlers: dict, extra=None, stdout=None) -> int:
    """Parse, run one command, print the one JSON object, return the exit status."""
    out = blank(vendor)
    words = list(sys.argv[1:] if argv is None else argv)
    if words and words[0] in COMMANDS:
        out["command"] = words[0]
    try:
        args = parse_arguments(words, extra)
        out["ip"] = args.ip
        if args.writes and not args.driver_stopped:
            raise Stop("refused", f"{args.command} writes to the sensor and needs --driver-stopped; nothing was sent")
        handlers[args.command](Run(args, out, sessions))
        raise RuntimeError("the command ended without an outcome")
    except Stop as stop:
        out["outcome"], out["message"] = stop.outcome, stop.message
    except NoAnswer as exc:  # after a change was sent, silence is "not read back", not "nothing there"
        out["outcome"], out["message"] = "unverified" if out.get("_wrote") else "no-answer", str(exc)
    except Exception as exc:  # noqa: BLE001 - the answer is always one JSON object
        out["outcome"] = "unverified" if out.get("_wrote") else "failed"
        out["message"] = f"{type(exc).__name__}: {exc}"
    out.pop("_wrote", None)
    out["exit_code"], out["sessions"] = EXIT[out["outcome"]], sessions.report()
    print(json.dumps(out), file=stdout or sys.stdout, flush=True)
    return out["exit_code"]


# ---- Ouster: HTTP -------------------------------------------------------------------------------------
# The interface part above is kept identical in the four driver repositories with a sensor-tool
# (Hesai, LSLiDAR, Ouster, Seyond); the sources of what follows are in the module text.

PORTS = {"http": 80}
SENSOR_INFO = "api/v1/sensor/metadata/sensor_info"  # SDK sensor_http_imp.cpp:19
ACTIVE = "api/v1/sensor/cmd/get_config_param?args=active"  # SDK sensor_http_imp.cpp:37
NETWORK = "api/v1/system/network"  # the manual, as recalled
PTP = "api/v1/time/ptp"  # the manual, as recalled
READS = (SENSOR_INFO, ACTIVE, NETWORK, PTP)  # every path this tool reads
SET_PARAM = "api/v1/sensor/cmd/set_config_param"  # SDK sensor_http_imp.cpp:32
REINITIALIZE = "api/v1/sensor/cmd/reinitialize"  # SDK sensor_http_imp.cpp:70
SAVE = "api/v1/sensor/cmd/save_config_params"  # SDK sensor_http_imp.cpp:74
OVERRIDE = "api/v1/system/network/ipv4/override"  # the manual, as recalled
PTP_PROFILE = "api/v1/time/ptp/profile"  # the manual, as recalled
WRITES = (SET_PARAM, REINITIALIZE, SAVE, OVERRIDE, PTP_PROFILE)  # every path this tool writes
# SDK types.cpp:50-54, timestamp_mode_strings
CLOCK = {"TIME_FROM_INTERNAL_OSC": "internal", "TIME_FROM_SYNC_PULSE_IN": "sync-pulse", "TIME_FROM_PTP_1588": "ptp"}
SETTABLE = ("udp_dest", "udp_ip", "udp_port_lidar", "udp_port_imu", "timestamp_mode")  # SDK types.cpp:794-804
# The firmware's PTP profiles: (the interface's profile name, transport, delay mechanism). Every one
# is taken to be domain 0. From the manual as recalled; "default-l2-relaxed" from its name alone.
PROFILES = {"default": ("1588v2", "UDPv4", "E2E"), "default-l2-relaxed": ("1588v2", "L2", "E2E"),
            "gptp": ("802.1AS", "L2", "P2P"), "automotive-slave": ("802.1AS automotive", "L2", "P2P")}
# The profile this tool writes for a transport and delay mechanism. UDPv4 with P2P has none.
CHOICE = {("UDPv4", "E2E"): "default", ("L2", "E2E"): "default-l2-relaxed", ("L2", "P2P"): "gptp"}


class Refusal(Exception):
    """The sensor answered an HTTP error."""


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *_args, **_kwargs):  # never follow a sensor to another host
        return None


def _request(run: Run, ip: str, method: str, path: str, body=None) -> str:
    """One HTTP request on its own connection: the answer's text. NoAnswer when nothing answers,
    Refusal for an error answer. Only `read` and `Writer` call this."""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect)
    request = urllib.request.Request(f"http://{ip}:{PORTS['http']}/{path}", method=method,
                                     data=None if body is None else body.encode(),
                                     headers={"Connection": "close", **({} if body is None else {"Content-Type": "application/json"})})
    run.sessions.opening("http")
    try:
        with opener.open(request, timeout=max(run.args.timeout, 0.2)) as reply:
            return reply.read(1024 * 1024).decode("utf-8", "replace")
    except urllib.error.HTTPError as exc:
        detail = exc.read(2048).decode("utf-8", "replace").strip()
        raise Refusal(f"HTTP {exc.code} for {method} {path.split('?')[0]}: {detail[:200] or exc.reason}") from None
    except (OSError, http.client.HTTPException) as exc:  # URLError is an OSError
        reason = getattr(exc, "reason", exc)
        raise NoAnswer(f"no HTTP answer at {ip}:{PORTS['http']} ({getattr(reason, 'strerror', None) or reason or type(exc).__name__})") from None
    finally:
        run.sessions.closed("http")


def read(run: Run, ip: str, path: str):
    """GET one of READS as JSON. This function cannot write: the method is fixed and so are the paths."""
    if path not in READS:
        raise ValueError(f"{path} is not a path this tool reads")
    text = _request(run, ip, "GET", path)
    try:
        return json.loads(text)
    except ValueError:
        raise NoAnswer(f"what answers at {ip}:{PORTS['http']} is not an Ouster sensor (no JSON for {path})") from None


class Writer:
    """Everything that changes a sensor. Made only in a run that writes with the driver stopped."""

    def __init__(self, run: Run) -> None:
        if not run.args.writes or not run.args.driver_stopped:
            raise Stop("refused", "a write needs --driver-stopped and no --dry-run; nothing was sent")
        self.run = run

    def _send(self, ip: str, method: str, path: str, body=None) -> str:
        if path.split("?")[0] not in WRITES:
            raise ValueError(f"{path} is not a path this tool writes")
        self.run.out["_wrote"] = True
        return _request(self.run, ip, method, path, body)

    def _command(self, ip: str, path: str, expected: str) -> None:
        try:
            answer = self._send(ip, "GET", path).strip()  # the SDK's execute(), sensor_http_imp.cpp:92-98: these commands are GETs
        except Refusal as exc:
            raise Stop("failed", f"the sensor refused: {exc}") from None
        if answer != expected:
            raise Stop("failed", f"the sensor answered {answer[:120]!r} to {path.split('?')[0]}, not {expected}")

    def configure(self, ip: str, active: dict, changes: dict) -> None:
        """SDK client.cpp:296-371, set_config: stage the active parameters with `changes`, make
        them effective, keep them."""
        if not set(changes) <= set(SETTABLE):
            raise ValueError(f"not a parameter this tool sets: {sorted(set(changes) - set(SETTABLE))}")
        # curl_easy_escape (SDK curl_client.h:33-39) and quote(safe="") leave the same characters as they are
        staged = urllib.parse.quote(json.dumps({**active, **changes}, separators=(",", ":")), safe="")
        self._command(ip, f"{SET_PARAM}?args=.+{staged}", '"set_config_param"')
        self._command(ip, REINITIALIZE, "{}")
        self._command(ip, SAVE, "{}")

    def override(self, ip: str, address: str, prefix: int) -> None:
        """The manual's PUT of the static address. The sensor resets its network just after it
        answers, so an answer that does not arrive is not a failure: the read-back decides."""
        try:
            self._send(ip, "PUT", OVERRIDE, json.dumps(f"{address}/{prefix}"))
        except Refusal as exc:
            raise Stop("failed", f"the sensor refused the address: {exc}") from None
        except NoAnswer:
            self.run.warn("the sensor did not answer the address change itself; the read-back decides")

    def profile(self, ip: str, name: str) -> None:
        """The manual's PUT of the PTP profile."""
        if name not in CHOICE.values():
            raise ValueError(f"{name} is not a PTP profile this tool writes")
        try:
            self._send(ip, "PUT", PTP_PROFILE, json.dumps(name))
        except Refusal as exc:
            raise Stop("failed", f"the sensor refused the PTP profile {name}: {exc}") from None


# ---- Ouster: reading ----------------------------------------------------------------------------------


def _port(value):
    return value if isinstance(value, int) and not isinstance(value, bool) and 0 < value < 65536 else None


def _text(value, limit: int = 64):
    return value.strip()[:limit] if isinstance(value, str) and value.strip() else None


def _interface(value):
    try:
        return ipaddress.IPv4Interface(value) if isinstance(value, str) and "/" in value else None
    except ValueError:
        return None


def read_all(run: Run, ip: str):
    """(identity, state). sensor_info and the configuration must answer; the network and the PTP
    state are the manual's paths, and a sensor that does not answer them is read without them."""
    try:
        info = read(run, ip, SENSOR_INFO)
    except Refusal as exc:
        # SDK sensor_http.cpp:30-51: firmware before 2.2 is read by other paths, or over TCP
        raise NoAnswer(f"what answers at {ip}:{PORTS['http']} is not an Ouster sensor with firmware 2.2 or later ({exc})") from None
    if not isinstance(info, dict) or not (info.get("prod_sn") or info.get("prod_line")):
        raise NoAnswer(f"what answers at {ip}:{PORTS['http']} is not an Ouster sensor (no sensor_info)")
    identity, state = blank_identity(VENDOR), blank_state()
    identity.update(model=_text(info.get("prod_line")), serial=_text(str(info.get("prod_sn") or "")), firmware=_text(info.get("image_rev")))
    try:
        active = read(run, ip, ACTIVE)
    except Refusal as exc:
        raise NoAnswer(f"the sensor at {ip} did not give its configuration: {exc}") from None
    if not isinstance(active, dict):
        raise NoAnswer(f"the sensor at {ip} did not give its configuration as an object")
    net, dest, ptp = state["net"], state["dest"], state["ptp"]
    net["ip"], net["static"] = ip, None  # it answers there
    try:
        network = read(run, ip, NETWORK)
        ipv4 = network.get("ipv4") if isinstance(network, dict) else None
        if not isinstance(ipv4, dict):
            raise Refusal("no ipv4 in the answer")
        identity["mac"] = (_text(network.get("ethaddr")) or "").lower() or None
        override = _text(ipv4.get("override"))
        net["static"] = override is not None and override.split("/")[0] == ip
        for candidate in (ipv4.get("override"), ipv4.get("addr"), ipv4.get("link_local")):  # "<address>/<prefix>"
            interface = _interface(candidate)
            if interface is not None and str(interface.ip) == ip:
                net["netmask"] = str(interface.netmask)
                break
    except (Refusal, NoAnswer) as exc:
        run.warn(f"the sensor's network settings were not read: {exc}")
    address = active.get("udp_dest") or active.get("udp_ip")  # SDK types.cpp:612-616: old firmware has udp_ip
    try:
        dest["ip"] = str(ipaddress.IPv4Address(address)) if isinstance(address, str) and address else None
    except ValueError:
        dest["ip"] = _text(address)  # a host name: shown, and never equal to an address asked for
    dest["data_port"], dest["imu_port"] = _port(active.get("udp_port_lidar")), _port(active.get("udp_port_imu"))
    mode = active.get("timestamp_mode")
    ptp["clock_source"] = CLOCK.get(mode, _text(mode)) if isinstance(mode, str) else None
    ptp["delay_mechanism"] = ptp["sensor_profile"] = None
    try:
        clock = read(run, ip, PTP)
        clock = clock if isinstance(clock, dict) else {}
        port = clock.get("port_data_set")
        ptp["status"] = _text(port.get("port_state"), 40) if isinstance(port, dict) else None
        ptp["sensor_profile"] = _text(clock.get("profile"), 40)
        if ptp["sensor_profile"] in PROFILES:
            ptp["profile"], ptp["transport"], ptp["delay_mechanism"] = PROFILES[ptp["sensor_profile"]]
            # The domain stays "not known": no call reads it. (hostd compares what is reported with
            # what was asked, so a domain taken from the table would be a claim nobody checked.)
    except (Refusal, NoAnswer):
        pass  # reported as not known; set-ptp then says what it cannot set
    state["channel"] = "http"
    state["dest"]["_active"] = active
    return identity, state


def probe(run: Run) -> None:
    identity, _state = read_all(run, run.args.ip)
    run.out["identity"] = identity
    raise Stop("reported", f"an Ouster sensor answers at {run.args.ip} over HTTP (serial number {identity['serial'] or 'not given'})")


def get_net(run: Run) -> None:
    identity, state = read_all(run, run.args.ip)
    if state["net"]["static"] is False:
        run.warn("the sensor's address is not a static one (no IPv4 override is set for it: DHCP or link-local)")
    run.out["identity"], run.out["after"] = identity, public(state)
    raise Stop("reported", f"the settings of the Ouster sensor at {run.args.ip}, read over HTTP")


# ---- Ouster: the set commands -------------------------------------------------------------------------


def reader(run: Run, ip: str):
    def read_state():
        try:
            return read_all(run, ip)[1]
        except (NoAnswer, Refusal):
            return None
    return read_state


def whole(run: Run, step: str, state: dict, reason: str) -> None:
    """The whole step cannot be done on this sensor: nothing is sent."""
    run.out["before"] = public(state)
    run.out["unsupported"] = [{"setting": step, "reason": reason}]
    raise Stop("unsupported", f"{step} cannot be set on this sensor: {reason}; nothing was sent")


def set_dest(run: Run) -> None:
    args, ip = run.args, run.args.ip
    if args.data_port >= 65535:
        raise Stop("refused", "the IMU port is the data port plus one, so the data port is at most 65534; nothing was sent")
    wanted = {"ip": args.dest_ip, "data_port": args.data_port, "imu_port": args.data_port + 1}
    run.out["identity"], state = read_all(run, ip)
    settle(run, "dest", wanted, state)
    changes = {"udp_dest": wanted["ip"], "udp_port_lidar": wanted["data_port"], "udp_port_imu": wanted["imu_port"]}
    if "udp_ip" in state["dest"]["_active"]:  # SDK client.cpp:341-343
        changes["udp_ip"] = wanted["ip"]
    Writer(run).configure(ip, state["dest"]["_active"], changes)
    after, good = await_state(time.monotonic() + args.verify_timeout, reader(run, ip), lambda s: not differing("dest", wanted, s["dest"]))
    finish(run, "dest", wanted, after, good, ip if after is not None else None)


def set_ptp(run: Run) -> None:
    args, ip = run.args, run.args.ip
    wanted = {"clock_source": "ptp"}
    run.out["identity"], state = read_all(run, ip)
    have = state["ptp"]
    if args.domain != 0:
        run.cannot("ptp.domain", "no call reads or sets the PTP domain; the firmware's profiles are taken to be domain 0")
    if have["sensor_profile"] in PROFILES:
        delay = args.delay_mechanism
        if (args.transport, delay) not in CHOICE:
            run.cannot("ptp.delay_mechanism", f"no PTP profile of the firmware is {args.transport} with {delay} delay; E2E is set")
            delay = "E2E"
        wanted.update(transport=args.transport, delay_mechanism=delay)
    else:  # what cannot be read is not written
        shown = f"shows the unknown PTP profile {have['sensor_profile']!r}" if have["sensor_profile"] else "does not show its PTP profile"
        for setting in ("transport", "delay_mechanism"):
            run.cannot(f"ptp.{setting}", f"this sensor {shown}; it was not written")
    changed = settle(run, "ptp", wanted, state)
    writer = Writer(run)
    if {"ptp.transport", "ptp.delay_mechanism"} & set(changed):
        name = CHOICE[(wanted["transport"], wanted["delay_mechanism"])]
        if PROFILES[name][0] != args.profile:
            run.warn(f"{wanted['transport']} with {wanted['delay_mechanism']} delay is the firmware's {name} profile ({PROFILES[name][0]}), not {args.profile}")
        writer.profile(ip, name)
    if "ptp.clock_source" in changed:
        writer.configure(ip, state["dest"]["_active"], {"timestamp_mode": "TIME_FROM_PTP_1588"})
    after, good = await_state(time.monotonic() + args.verify_timeout, reader(run, ip), lambda s: not differing("ptp", wanted, s["ptp"]))
    finish(run, "ptp", wanted, after, good, ip if after is not None else None)


def set_net(run: Run) -> None:
    args, old, new = run.args, run.args.ip, run.args.new_ip
    wanted = {"ip": new, "netmask": args.netmask, "static": True}
    identity, state = read_all(run, old)
    run.out["identity"] = identity
    if state["net"]["static"] is None:
        whole(run, "net", state, "it does not show its network settings, and what cannot be read is not written")
    if args.gateway != "0.0.0.0":
        run.cannot("net.gateway", "no call reads or sets an Ouster's gateway; it was not written")
    if differing("net", {"ip": new, "netmask": args.netmask}, state["net"]):
        check_new_address(new, args.netmask, args.gateway)
    settle(run, "net", wanted, state)
    prefix = ipaddress.IPv4Network(f"{new}/{args.netmask}", strict=False).prefixlen
    Writer(run).override(old, new, prefix)

    def at_new():
        try:
            there, found = read_all(run, new)
        except (NoAnswer, Refusal):
            return None
        if identity["serial"] and there["serial"] and there["serial"] != identity["serial"]:
            raise Stop("failed", f"another sensor answers at {new} (serial number {there['serial']}, not {identity['serial']}); "
                       f"the address was sent to the sensor at {old}")
        return found
    after, good = await_state(time.monotonic() + args.verify_timeout, at_new, lambda s: not differing("net", wanted, s["net"]))
    if good:
        finish(run, "net", wanted, after, True, new)
    here = new if after is not None else old if reader(run, old)() is not None else None
    run.out["after"], run.out["verified"], run.out["answers_at"] = public(after), False, here
    raise Stop("unverified", f"the new address was sent, and the sensor did not answer at {new} with it within {args.verify_timeout:g} s"
               + (f"; it answers at {here}" if here else "; it answers at neither address")
               + ". There is no restart call here: if the sensor needs one to take the address, power-cycle it")


HANDLERS = {"probe": probe, "get-net": get_net, "set-net": set_net, "set-dest": set_dest, "set-ptp": set_ptp}


def main(argv=None, stdout=None) -> int:
    return run_tool(argv, VENDOR, Sessions(("http",)), HANDLERS, None, stdout)


if __name__ == "__main__":
    sys.exit(main())

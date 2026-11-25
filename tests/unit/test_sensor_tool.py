"""sensor-tool (docker/sensor_tool.py) against the fake Ouster sensor's HTTP interface
(docker/smoke/fake_sensor.py) on loopback addresses: the interface's rules, each with its test, and
every outcome of every command. No real sensor is contacted, and nothing leaves this host."""

from __future__ import annotations

import contextlib
import copy
import hashlib
import io
import ipaddress
import json
import subprocess
import sys
import threading
import types
import urllib.parse
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

import fake_sensor
import ouster_packets as op
import sensor_tool

TOOL = Path(__file__).resolve().parents[2] / "docker" / "sensor_tool.py"
A, B = "127.0.60.1", "127.0.60.2"
MASK = "255.0.0.0"
KEYS = {"sensor_tool", "vendor", "command", "ip", "outcome", "exit_code", "message", "identity", "before", "wanted", "after",
        "changed", "unsupported", "reboot", "verified", "answers_at", "sessions", "warnings"}
NET = ["--new-ip", B, "--netmask", MASK, "--gateway", "0.0.0.0"]
DEST = ["--dest-ip", "127.0.0.77", "--data-port", "2368"]
PTP = ["--transport", "UDPv4", "--domain", "0", "--delay-mechanism", "E2E", "--timescale", "TAI", "--profile", "1588v2", "--clock-source", "ptp"]
SETS = {"set-net": NET, "set-dest": DEST, "set-ptp": PTP}
STOPPED = ["--driver-stopped", "--verify-timeout", "1.5"]
# What metrolla-edge's hostd accepts (sensor_program.TOOL_OUTCOMES; tests/support/fake_sensor_tool.py EXIT).
HOSTD_EXIT = {"reported": 0, "set": 0, "unchanged": 0, "changes": 2, "refused": 1, "failed": 1, "no-answer": 1, "unsupported": 3, "unverified": 5}
# The interface part is the same text in the four driver repositories (Hesai, LSLiDAR, Ouster, Seyond).
INTERFACE_SHA256 = "a5a54aa59c939cd51e60bef9de6b49de4934e0f8750c52d15cf6d225c6c9ea58"


@pytest.fixture(scope="module")
def recording():
    return op.load()


@pytest.fixture(autouse=True)
def quick(monkeypatch):
    monkeypatch.setattr(sensor_tool, "PAUSE", 0.05)
    monkeypatch.setattr(sensor_tool, "ALLOW_LOOPBACK", True)


@pytest.fixture
def fake(recording, monkeypatch):
    """A factory of fake sensors: fake(at=A, listen=(A, B), port=None, serial=None, **attributes)."""
    with contextlib.ExitStack() as stack:
        def make(at=A, listen=(A, B), port=None, serial=None, faults=(), **attributes):
            rec = recording
            if serial is not None:
                rec = copy.copy(recording)
                rec.meta = {**recording.meta, "prod_sn": serial}
            sensor = fake_sensor.Sensor(rec)
            sensor.address, sensor.prefix, sensor.override, sensor.faults = at, 8, f"{at}/8", set(faults)
            for name, value in attributes.items():
                setattr(sensor, name, value)
            for address in listen:
                server = ThreadingHTTPServer((address, port or 0), fake_sensor.make_handler(sensor))
                server.daemon_threads = True
                port = server.server_address[1]
                threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True).start()
                stack.callback(server.server_close)
                stack.callback(server.shutdown)
            sensor.port = port
            monkeypatch.setitem(sensor_tool.PORTS, "http", port)
            return sensor
        yield make


def hostd_accepts(status: int, output: str, command: str) -> dict:
    """The checks metrolla-edge's hostd makes of a tool's answer (sensor_program.check_answer)."""
    assert len(output) <= 64 * 1024
    lines = [line for line in output.splitlines() if line.strip()]
    assert len(lines) == 1, lines
    answer = json.loads(lines[0])
    assert isinstance(answer, dict) and answer["sensor_tool"] == 1
    assert answer["outcome"] in HOSTD_EXIT and HOSTD_EXIT[answer["outcome"]] == status == answer["exit_code"]
    assert answer["vendor"] == "ouster" and answer["command"] == command
    assert set(answer) == KEYS and isinstance(answer["message"], str) and answer["message"]
    for key in ("before", "wanted", "after"):
        assert answer[key] is None or isinstance(answer[key], dict)
    for key in ("before", "after"):
        if answer[key] is not None:
            assert set(answer[key]) == {"net", "dest", "ptp", "channel"} and answer[key]["channel"] == "http"
            assert {"ip", "netmask", "gateway"} <= set(answer[key]["net"]) and {"ip", "data_port"} <= set(answer[key]["dest"])
            assert {"clock_source", "profile", "transport", "domain", "status"} <= set(answer[key]["ptp"])
    assert answer["identity"] is None or set(answer["identity"]) == {"vendor", "model", "serial", "mac", "firmware"}
    assert all(isinstance(c, str) for c in answer["changed"]) and all(isinstance(w, str) for w in answer["warnings"])
    assert all(set(u) == {"setting", "reason"} for u in answer["unsupported"])
    assert answer["reboot"] == "none" and answer["verified"] in (True, False, None)
    assert answer["answers_at"] is None or ipaddress.IPv4Address(answer["answers_at"])
    assert answer["sessions"]["max_concurrent"] <= 1
    return answer


def run(sensor, *argv, expect=None):
    """One tool run: (answer, exit status). The answer is checked as hostd checks it, every time."""
    printed = io.StringIO()
    status = sensor_tool.main([str(word) for word in argv] + ["--timeout", "1"], stdout=printed)
    answer = hostd_accepts(status, printed.getvalue(), argv[0])
    if sensor is not None:
        assert sensor.max_open <= 1  # never two connections at once
        seen = len(sensor.requests)
        assert answer["sessions"]["http"] >= seen - getattr(sensor, "_seen", 0)  # every connection is counted
        sensor._seen = seen
    if expect is not None:
        assert answer["outcome"] == expect, answer
    return answer, status


def writes(sensor) -> list:
    """The requests the sensor got that are not one of the tool's four reads (the fake logs
    "<method> /<path>[ <body>]")."""
    return [r for r in sensor.requests if r.removeprefix("GET /") not in sensor_tool.READS]


def staged_blob(request: str) -> dict:
    return json.loads(urllib.parse.unquote(request.partition("args=.+")[2]))


# --- reading ----------------------------------------------------------------------------------------


def test_probe_and_get_net_beside_the_running_driver(fake, recording):
    sensor = fake(at=A)
    answer, status = run(sensor, "probe", "--ip", A, expect="reported")  # no --driver-stopped: HTTP only
    assert status == 0
    assert answer["identity"] == {"vendor": "ouster", "model": "OS-2-128", "serial": recording.meta["prod_sn"],
                                  "mac": "bc:0f:a7:00:01:2c", "firmware": recording.meta["image_rev"]}
    answer, _ = run(sensor, "get-net", "--ip", A, "--model", "OS-2-128", "--dev-port", "7502", expect="reported")
    assert answer["after"] == {
        "net": {"ip": A, "netmask": MASK, "gateway": None, "static": True},
        "dest": {"ip": None, "data_port": 7502, "imu_port": 7503},
        "ptp": {"clock_source": "internal", "profile": "1588v2", "transport": "UDPv4", "domain": None, "status": "SLAVE",
                "delay_mechanism": "E2E", "sensor_profile": "default"},
        "channel": "http"}
    assert answer["sessions"] == {"http": 4, "max_concurrent": 1} and answer["warnings"] == []
    assert writes(sensor) == [] and len(sensor.requests) == 8


def test_an_address_that_is_not_static_is_said(fake):
    answer, _ = run(fake(at=A, override=None), "get-net", "--ip", A, expect="reported")
    assert answer["after"]["net"] == {"ip": A, "netmask": MASK, "gateway": None, "static": False}
    assert "not a static one" in answer["warnings"][0]


def test_a_sensor_without_the_manuals_paths_is_read_without_them(fake):
    answer, _ = run(fake(at=A, faults={"no-network", "no-ptp"}), "get-net", "--ip", A, expect="reported")
    assert answer["after"]["net"] == {"ip": A, "netmask": None, "gateway": None, "static": None}
    assert answer["after"]["ptp"] == {"clock_source": "internal", "profile": None, "transport": None, "domain": None, "status": None,
                                      "delay_mechanism": None, "sensor_profile": None}
    assert answer["identity"]["mac"] is None and "network settings were not read" in answer["warnings"][0]


def test_no_answer(fake):
    sensor = fake(at=B)  # listens on A too, and is not there
    for command in ("probe", "get-net"):
        answer, status = run(sensor, command, "--ip", A, expect="no-answer")
        assert status == 1 and answer["identity"] is None and answer["after"] is None
    answer, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="no-answer")
    assert status == 1 and sensor.requests == []


def test_something_else_on_port_80_is_not_an_ouster(fake, monkeypatch):
    sensor = fake(at=A)
    monkeypatch.setattr(sensor.rec, "meta", {**sensor.rec.meta, "prod_sn": "", "prod_line": ""})
    answer, _ = run(sensor, "probe", "--ip", A, expect="no-answer")
    assert "not an Ouster sensor" in answer["message"]


# --- reading and writing are separate -----------------------------------------------------------------


def test_reads_and_dry_runs_send_no_write(fake):
    sensor = fake(at=A)
    for stopped in ([], ["--driver-stopped"]):
        run(sensor, "probe", "--ip", A, *stopped, expect="reported")
        run(sensor, "get-net", "--ip", A, *stopped, expect="reported")
        for command, values in SETS.items():
            answer, status = run(sensor, command, "--ip", A, *values, "--dry-run", *stopped, expect="changes")
            assert status == 2 and answer["changed"] and answer["after"] is None and answer["verified"] is None
    assert writes(sensor) == [] and sensor.reinits == 0 and sensor.persisted == 0 and sensor.address == A


def test_the_read_function_cannot_write(fake):
    sensor = fake(at=A)
    args = types.SimpleNamespace(timeout=1.0, writes=False, driver_stopped=True, dry_run=True)
    a_run = sensor_tool.Run(args, sensor_tool.blank("ouster"), sensor_tool.Sessions(("http",)))
    for path in (*sensor_tool.WRITES, sensor_tool.SET_PARAM + "?args=udp_dest+1.2.3.4", "api/v1/sensor/cmd/set_udp_dest_auto",
                 "api/v1/sensor/cmd/get_config_param?args=staged", "api/v1/system/firmware"):
        with pytest.raises(ValueError, match="not a path this tool reads"):
            sensor_tool.read(a_run, A, path)
    with pytest.raises(sensor_tool.Stop) as refused:  # a dry run cannot have a Writer
        sensor_tool.Writer(a_run)
    assert refused.value.outcome == "refused"
    args.writes, args.driver_stopped = True, False
    with pytest.raises(sensor_tool.Stop):  # nor a run with the driver running
        sensor_tool.Writer(a_run)
    args.driver_stopped = True
    writer = sensor_tool.Writer(a_run)
    for send in (lambda: writer._send(A, "DELETE", "api/v1/system/network/ipv4/override/x"),
                 lambda: writer._send(A, "GET", "api/v1/sensor/cmd/set_udp_dest_auto"),
                 lambda: writer.profile(A, "automotive-slave"),
                 lambda: writer.configure(A, {}, {"lidar_mode": "512x10"})):
        with pytest.raises(ValueError):
            send()
    assert sensor.requests == []
    assert set(sensor_tool.READS).isdisjoint(sensor_tool.WRITES)


@pytest.mark.parametrize("command", list(SETS))
def test_every_write_needs_the_driver_stopped(fake, command):
    sensor = fake(at=A)
    answer, status = run(sensor, command, "--ip", A, *SETS[command], expect="refused")
    assert status == 1 and "--driver-stopped" in answer["message"] and answer["sessions"]["http"] == 0
    assert sensor.requests == []  # nothing was sent, not even a read


# --- set-dest ---------------------------------------------------------------------------------------


def test_set_dest_changes_only_the_destination_and_is_idempotent(fake):
    sensor = fake(at=A)
    sensor.staged["lidar_mode"] = "512x10"  # something staged earlier by someone else: it must not come along
    before = copy.deepcopy(sensor.active)
    answer, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="set")
    assert status == 0 and answer["verified"] is True and answer["answers_at"] == A and answer["unsupported"] == []
    assert answer["changed"] == ["dest.ip", "dest.data_port", "dest.imu_port"]
    assert answer["before"]["dest"] == {"ip": None, "data_port": 7502, "imu_port": 7503}
    assert answer["wanted"] == {"dest": {"ip": "127.0.0.77", "data_port": 2368, "imu_port": 2369}} == {"dest": answer["after"]["dest"]}
    sent = writes(sensor)
    assert [r.partition("?")[0] for r in sent] == ["GET /api/v1/sensor/cmd/set_config_param", "GET /api/v1/sensor/cmd/reinitialize",
                                                  "GET /api/v1/sensor/cmd/save_config_params"]
    assert staged_blob(sent[0]) == {**before, "udp_dest": "127.0.0.77", "udp_port_lidar": 2368, "udp_port_imu": 2369}
    assert sensor.active == {**before, "udp_dest": "127.0.0.77", "udp_port_lidar": 2368, "udp_port_imu": 2369}
    assert sensor.reinits == 1 and sensor.persisted == 1
    again, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="unchanged")
    assert status == 0 and again["changed"] == [] and again["verified"] is True and writes(sensor) == sent
    dry, _ = run(sensor, "set-dest", "--ip", A, *DEST, "--dry-run", expect="unchanged")
    assert dry["exit_code"] == 0


def test_set_dest_of_the_port_alone(fake):
    sensor = fake(at=A)
    sensor.active["udp_dest"] = sensor.staged["udp_dest"] = "127.0.0.77"
    answer, _ = run(sensor, "set-dest", "--ip", A, "--dest-ip", "127.0.0.77", "--data-port", "7600", *STOPPED, expect="set")
    assert answer["changed"] == ["dest.data_port", "dest.imu_port"] and sensor.active["udp_port_imu"] == 7601


def test_set_dest_writes_the_old_udp_ip_too_where_the_sensor_has_it(fake):
    sensor = fake(at=A)
    sensor.active["udp_ip"] = sensor.staged["udp_ip"] = ""
    run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="set")
    assert sensor.active["udp_ip"] == sensor.active["udp_dest"] == "127.0.0.77"


def test_the_last_port_has_no_imu_port(fake):
    sensor = fake(at=A)
    answer, status = run(sensor, "set-dest", "--ip", A, "--dest-ip", "127.0.0.77", "--data-port", "65535", *STOPPED, expect="refused")
    assert status == 1 and sensor.requests == []


def test_a_refused_write_is_failed(fake):
    sensor = fake(at=A, faults={"refuse-config"})
    answer, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="failed")
    assert status == 1 and "the sensor refused" in answer["message"] and "HTTP 400" in answer["message"]
    assert sensor.reinits == 0 and sensor.persisted == 0  # the run stopped at the refusal


def test_a_write_the_sensor_does_not_take_is_unverified(fake):
    sensor = fake(at=A, faults={"ignore-config"})
    answer, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="unverified")
    assert status == 5 and answer["verified"] is False and answer["answers_at"] == A
    assert answer["after"]["dest"]["data_port"] == 7502 and "still different: dest.ip, dest.data_port, dest.imu_port" in answer["message"]


def test_a_sensor_that_goes_silent_after_the_change_is_unverified(fake, monkeypatch):
    sensor = fake(at=A)
    real = sensor_tool.Writer.configure

    def configure(self, ip, active, changes):
        real(self, ip, active, changes)
        sensor.faults.add("silent")  # the change was taken, and the sensor answers no more
    monkeypatch.setattr(sensor_tool.Writer, "configure", configure)
    answer, status = run(sensor, "set-dest", "--ip", A, *DEST, *STOPPED, expect="unverified")
    assert status == 5 and answer["verified"] is False and answer["answers_at"] is None and answer["after"] is None


# --- set-ptp ----------------------------------------------------------------------------------------


def test_set_ptp_sets_the_clock_source_and_is_idempotent(fake):
    sensor = fake(at=A)
    before = copy.deepcopy(sensor.active)
    answer, status = run(sensor, "set-ptp", "--ip", A, *PTP, *STOPPED, expect="set")
    assert status == 0 and answer["changed"] == ["ptp.clock_source"] and answer["unsupported"] == [] and answer["warnings"] == []
    assert answer["wanted"] == {"ptp": {"clock_source": "ptp", "transport": "UDPv4", "delay_mechanism": "E2E"}}
    assert answer["before"]["ptp"]["clock_source"] == "internal" and answer["after"]["ptp"]["clock_source"] == "ptp"
    assert sensor.active == {**before, "timestamp_mode": "TIME_FROM_PTP_1588"} and sensor.ptp_profile == "default"
    assert not any(r.startswith("PUT") for r in sensor.requests) and sensor.persisted == 1
    sent = writes(sensor)
    again, status = run(sensor, "set-ptp", "--ip", A, *PTP, *STOPPED, expect="unchanged")
    assert status == 0 and writes(sensor) == sent


def ptp(transport: str, delay: str, domain: int = 0) -> list:
    return ["--transport", transport, "--domain", domain, "--delay-mechanism", delay, "--timescale", "TAI"]


@pytest.mark.parametrize("transport, delay, profile", [("L2", "P2P", "gptp"), ("L2", "E2E", "default-l2-relaxed")])
def test_set_ptp_writes_the_firmware_profile_that_fits(fake, transport, delay, profile):
    sensor = fake(at=A)
    answer, _ = run(sensor, "set-ptp", "--ip", A, *ptp(transport, delay), *STOPPED, expect="set")
    assert sensor.ptp_profile == profile and answer["unsupported"] == []
    assert set(answer["changed"]) == {"ptp.clock_source", "ptp.transport"} | ({"ptp.delay_mechanism"} if delay == "P2P" else set())
    assert (answer["after"]["ptp"]["transport"], answer["after"]["ptp"]["delay_mechanism"], answer["after"]["ptp"]["domain"]) == (transport, delay, None)
    assert f'PUT /api/v1/time/ptp/profile "{profile}"' in sensor.requests
    assert bool(answer["warnings"]) == (profile == "gptp")  # 802.1AS, not the 1588v2 that was asked for
    run(sensor, "set-ptp", "--ip", A, *ptp(transport, delay), *STOPPED, expect="unchanged")
    back, _ = run(sensor, "set-ptp", "--ip", A, *PTP, *STOPPED, expect="set")  # and back to UDPv4: the profile alone
    assert sensor.ptp_profile == "default" and "ptp.clock_source" not in back["changed"] and sensor.reinits == 1


def test_a_profile_that_already_fits_is_kept(fake):
    sensor = fake(at=A, ptp_profile="automotive-slave")
    sensor.active["timestamp_mode"] = "TIME_FROM_PTP_1588"
    answer, _ = run(sensor, "set-ptp", "--ip", A, *ptp("L2", "P2P"), *STOPPED, expect="unchanged")
    assert writes(sensor) == [] and answer["after"]["ptp"]["profile"] == "802.1AS automotive"


def test_peer_to_peer_over_udp_is_said_and_the_rest_is_set(fake):
    sensor = fake(at=A, ptp_profile="gptp")
    answer, status = run(sensor, "set-ptp", "--ip", A, *ptp("UDPv4", "P2P"), *STOPPED, expect="set")
    assert status == 0 and [u["setting"] for u in answer["unsupported"]] == ["ptp.delay_mechanism"]
    assert sensor.ptp_profile == "default" and sensor.active["timestamp_mode"] == "TIME_FROM_PTP_1588"
    again, _ = run(sensor, "set-ptp", "--ip", A, *ptp("UDPv4", "P2P"), *STOPPED, expect="unchanged")
    assert [u["setting"] for u in again["unsupported"]] == ["ptp.delay_mechanism"]  # said every time, never skipped silently


def test_another_domain_is_said(fake):
    sensor = fake(at=A)
    answer, _ = run(sensor, "set-ptp", "--ip", A, *ptp("UDPv4", "E2E", 3), *STOPPED, expect="set")
    assert [u["setting"] for u in answer["unsupported"]] == ["ptp.domain"] and answer["after"]["ptp"]["domain"] is None


@pytest.mark.parametrize("how", ["no-ptp", "unknown"])
def test_a_profile_that_cannot_be_read_is_not_written(fake, how):
    sensor = fake(at=A, faults={"no-ptp"}) if how == "no-ptp" else fake(at=A, ptp_profile="something-new")
    answer, _ = run(sensor, "set-ptp", "--ip", A, *ptp("L2", "P2P"), *STOPPED, expect="set")
    assert [u["setting"] for u in answer["unsupported"]] == ["ptp.transport", "ptp.delay_mechanism"]
    assert answer["changed"] == ["ptp.clock_source"] and not any(r.startswith("PUT") for r in sensor.requests)


def test_a_refused_profile_is_failed_and_one_not_taken_is_unverified(fake):
    sensor = fake(at=A, faults={"refuse-profile"})
    answer, status = run(sensor, "set-ptp", "--ip", A, *ptp("L2", "P2P"), *STOPPED, expect="failed")
    assert status == 1 and "refused the PTP profile gptp" in answer["message"] and sensor.reinits == 0
    sensor = fake(at=A, faults={"ignore-profile"})
    answer, status = run(sensor, "set-ptp", "--ip", A, *ptp("L2", "P2P"), *STOPPED, expect="unverified")
    assert status == 5 and "still different: ptp.transport, ptp.delay_mechanism" in answer["message"]


# --- set-net ----------------------------------------------------------------------------------------


def test_set_net_moves_the_sensor_and_is_idempotent(fake, recording):
    sensor = fake(at=A)
    answer, status = run(sensor, "set-net", "--ip", A, *NET, "--no-reboot", *STOPPED, expect="set")
    assert status == 0 and answer["changed"] == ["net.ip"] and answer["verified"] is True and answer["answers_at"] == B
    assert answer["before"]["net"]["ip"] == A and answer["after"]["net"] == {"ip": B, "netmask": MASK, "gateway": None, "static": True}
    assert answer["identity"]["serial"] == recording.meta["prod_sn"] and answer["unsupported"] == [] and answer["reboot"] == "none"
    assert writes(sensor) == [f'PUT /api/v1/system/network/ipv4/override "{B}/8"'] and sensor.address == B
    again, status = run(sensor, "set-net", "--ip", B, *NET, *STOPPED, expect="unchanged")
    assert status == 0 and len(writes(sensor)) == 1
    run(sensor, "set-net", "--ip", A, *NET, *STOPPED, expect="no-answer")  # nothing is at the old address now


def test_set_net_of_the_netmask_alone(fake):
    sensor = fake(at=A)
    answer, _ = run(sensor, "set-net", "--ip", A, "--new-ip", A, "--netmask", "255.255.0.0", "--gateway", "0.0.0.0", *STOPPED, expect="set")
    assert answer["changed"] == ["net.netmask"] and sensor.override == f"{A}/16" and answer["answers_at"] == A


def test_an_address_that_is_not_static_is_made_static(fake):
    sensor = fake(at=A, override=None)  # the address it has came from DHCP
    answer, _ = run(sensor, "set-net", "--ip", A, "--new-ip", A, "--netmask", MASK, "--gateway", "0.0.0.0", *STOPPED, expect="set")
    assert answer["changed"] == ["net.static"] and sensor.override == f"{A}/8"


def test_the_gateway_cannot_be_set_and_is_said(fake):
    sensor = fake(at=A)
    answer, status = run(sensor, "set-net", "--ip", A, "--new-ip", B, "--netmask", MASK, "--gateway", "127.0.0.254", *STOPPED, expect="set")
    assert status == 0 and [u["setting"] for u in answer["unsupported"]] == ["net.gateway"] and answer["after"]["net"]["gateway"] is None
    again, _ = run(sensor, "set-net", "--ip", B, "--new-ip", B, "--netmask", MASK, "--gateway", "127.0.0.254", *STOPPED, expect="unchanged")
    assert [u["setting"] for u in again["unsupported"]] == ["net.gateway"]


def test_what_cannot_be_set_at_all_is_unsupported_and_nothing_is_sent(fake):
    sensor = fake(at=A, faults={"no-network"})
    for extra in (STOPPED, ["--dry-run"]):
        answer, status = run(sensor, "set-net", "--ip", A, *NET, *extra, expect="unsupported")
        assert status == 3 and answer["unsupported"][0]["setting"] == "net" and answer["changed"] == [] and answer["before"] is not None
    assert writes(sensor) == []


def test_an_address_the_host_could_not_reach_is_refused_before_anything_is_sent(fake):
    sensor = fake(at=A)
    for extra in (STOPPED, ["--dry-run"]):
        answer, status = run(sensor, "set-net", "--ip", A, "--new-ip", "192.0.2.50", "--netmask", "255.255.255.0", "--gateway", "0.0.0.0",
                             *extra, expect="refused")
        assert status == 1 and "nothing was sent" in answer["message"]
    for new, mask, gateway in ((B, MASK, "10.0.0.1"), ("127.255.255.255", MASK, "0.0.0.0"), ("224.0.0.5", MASK, "0.0.0.0")):
        run(sensor, "set-net", "--ip", A, "--new-ip", new, "--netmask", mask, "--gateway", gateway, *STOPPED, expect="refused")
    assert writes(sensor) == []


def test_a_refused_address_is_failed(fake):
    sensor = fake(at=A, faults={"refuse-override"})
    answer, status = run(sensor, "set-net", "--ip", A, *NET, *STOPPED, expect="failed")
    assert status == 1 and "refused the address" in answer["message"] and sensor.address == A


def test_an_address_the_sensor_does_not_take_is_unverified_and_says_where_it_answers(fake):
    sensor = fake(at=A, faults={"ignore-override"})
    answer, status = run(sensor, "set-net", "--ip", A, *NET, *STOPPED, expect="unverified")
    assert status == 5 and answer["verified"] is False and answer["answers_at"] == A and "power-cycle" in answer["message"]


def test_a_sensor_that_vanishes_after_the_change_is_unverified(fake):
    sensor = fake(at=A, faults={"vanish"})
    answer, status = run(sensor, "set-net", "--ip", A, *NET, *STOPPED, expect="unverified")
    assert status == 5 and answer["answers_at"] is None and answer["after"] is None and "neither address" in answer["message"]


def test_another_sensor_at_the_new_address_fails(fake):
    other = fake(at=B, listen=(B,), serial="992000000002")
    sensor = fake(at=A, listen=(A,), port=other.port)
    answer, status = run(sensor, "set-net", "--ip", A, *NET, *STOPPED, expect="failed")
    assert status == 1 and "another sensor answers at" in answer["message"] and writes(other) == []


# --- the interface ----------------------------------------------------------------------------------


@pytest.mark.parametrize("argv, command", [
    ([], None), (["probe"], "probe"), (["erase", "--ip", A], None), (["probe", "--ip", "sensor.local"], "probe"),
    (["set-net", "--ip", A, "--new-ip", B, "--netmask", "255.0.255.0", "--gateway", "0.0.0.0"], "set-net"),
    (["set-dest", "--ip", A, "--dest-ip", B, "--data-port", "0"], "set-dest"),
    (["set-ptp", "--ip", A, "--transport", "CAN", "--domain", "0", "--delay-mechanism", "E2E", "--timescale", "TAI"], "set-ptp"),
    (["get-net", "--ip", A, "--help"], "get-net"),
])
def test_it_prints_one_json_object_also_for_a_usage_error(argv, command):
    done = subprocess.run([sys.executable, "-I", str(TOOL), *argv], capture_output=True, text=True, timeout=30)
    answer = json.loads(done.stdout)
    assert done.stdout.count("\n") == 1 and done.stderr == "" and done.returncode == 1
    assert set(answer) == KEYS and (answer["sensor_tool"], answer["vendor"], answer["command"]) == (1, "ouster", command)
    assert (answer["outcome"], answer["exit_code"]) == ("refused", 1) and answer["message"].startswith("usage: ")


def test_the_program_itself_reads_a_sensor(fake):
    """As the host runs it: the file, not the module. Port 80 is the only thing changed, by a wrapper."""
    sensor = fake(at=A)
    wrapper = f"import runpy, sys; sys.argv = ['sensor-tool', 'get-net', '--ip', '{A}', '--timeout', '2']; " \
              f"m = runpy.run_path({str(TOOL)!r}); m['PORTS']['http'] = {sensor.port}; sys.exit(m['main']())"
    done = subprocess.run([sys.executable, "-I", "-c", wrapper], capture_output=True, text=True, timeout=30)
    answer = hostd_accepts(done.returncode, done.stdout, "get-net")
    assert answer["outcome"] == "reported" and answer["after"]["net"]["ip"] == A and done.stderr == ""


def test_a_fault_of_the_tool_is_one_json_object_too(fake, monkeypatch):
    sensor = fake(at=A)
    monkeypatch.setitem(sensor_tool.HANDLERS, "probe", lambda run: 1 / 0)
    answer, status = run(sensor, "probe", "--ip", A, expect="failed")
    assert status == 1 and answer["message"].startswith("ZeroDivisionError")


def test_a_second_connection_while_one_is_open_is_refused_by_the_tool_itself():
    sessions = sensor_tool.Sessions(("http",))
    sessions.opening("http")
    with pytest.raises(RuntimeError, match="second control connection"):
        sessions.opening("http")
    sessions.closed("http")
    sessions.opening("http")
    assert sessions.report() == {"http": 2, "max_concurrent": 1}


def test_outcomes_and_exit_statuses_are_the_ones_hostd_expects():
    assert sensor_tool.EXIT == HOSTD_EXIT and sensor_tool.INTERFACE == 1 and sensor_tool.VENDOR == "ouster"
    assert sensor_tool.COMMANDS == ("probe", "get-net", "set-net", "set-dest", "set-ptp")
    assert set(sensor_tool.blank("ouster")) == KEYS


def test_hostd_arguments_are_accepted(fake):
    """The argument lists metrolla-edge's sensor_program.tool_arguments builds, word for word."""
    sensor = fake(at=A)
    base = ["--ip", A, "--model", "OS-1-64"]
    run(sensor, "probe", *base, expect="reported")
    run(sensor, "get-net", *base, "--driver-stopped", expect="reported")
    run(sensor, "set-ptp", *base, "--driver-stopped", "--transport", "UDPv4", "--domain", "0", "--delay-mechanism", "E2E", "--timescale", "TAI",
        "--profile", "1588v2", "--clock-source", "ptp", "--verify-timeout", "60", expect="set")
    run(sensor, "set-dest", *base, "--driver-stopped", "--dest-ip", "127.0.0.77", "--data-port", "2368", "--verify-timeout", "60", expect="set")
    run(sensor, "set-net", *base, "--new-ip", B, "--netmask", MASK, "--gateway", "0.0.0.0", "--verify-timeout", "60", "--dry-run", expect="changes")
    run(sensor, "set-net", *base, "--driver-stopped", "--new-ip", B, "--netmask", MASK, "--gateway", "0.0.0.0", "--verify-timeout", "60", expect="set")
    final, _ = run(sensor, "get-net", "--ip", B, "--model", "OS-1-64", "--driver-stopped", expect="reported")
    assert final["after"]["net"]["ip"] == B and final["after"]["dest"]["data_port"] == 2368 and final["after"]["ptp"]["clock_source"] == "ptp"


def test_the_interface_part_is_the_shared_text():
    text = TOOL.read_text()
    start = text.index("# ---- the interface: this part is the same text in every driver repository's sensor-tool")
    end = text.index("# ---- Ouster: HTTP")
    assert hashlib.sha256(text[start:end].encode()).hexdigest() == INTERFACE_SHA256


def test_the_sources_cited_are_where_the_tool_says():
    """The SDK lines the tool cites still say what it took from them (the submodule is pinned)."""
    sdk = TOOL.parents[1] / "ouster-ros" / "ouster-sdk" / "ouster_client" / "src"
    http = (sdk / "sensor_http_imp.cpp").read_text().splitlines()
    assert sensor_tool.SENSOR_INFO in http[18] and sensor_tool.ACTIVE in http[36] and sensor_tool.SET_PARAM in http[31]
    assert '"\\"set_config_param\\""' in http[32] and sensor_tool.REINITIALIZE in http[69] and sensor_tool.SAVE in http[73]
    types_cpp = (sdk / "types.cpp").read_text().splitlines()
    assert all(f'"{name}"' in "".join(types_cpp[49:54]) for name in sensor_tool.CLOCK)
    assert "udp_dest" in types_cpp[611] and "udp_ip" in types_cpp[613] and "udp_port_lidar" in types_cpp[621]
    assert "udp_port_imu" in types_cpp[623] and "timestamp_mode" in types_cpp[625]
    assert all(f'root["{key}"]' in "".join(types_cpp[793:804]) for key in sensor_tool.SETTABLE if key != "udp_ip")
    client = (sdk / "client.cpp").read_text().splitlines()
    assert 'set_config_param(".", config_params_str)' in client[358] and "reinitialize()" in client[360] and "save_config_params()" in client[365]
    assert 'config_params["udp_ip"]' in client[341]
    info = (sdk / "sensor_info.cpp").read_text().splitlines()
    assert all(key in "".join(info[253:259]) for key in ("image_rev", "prod_line", "prod_sn"))

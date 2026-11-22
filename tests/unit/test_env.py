"""Unit tests of the environment contract (docker/ouster_env.py): parsing, defaults, refusals,
rendering against the vendor parameter file, the exec line, and the run/check commands."""

from __future__ import annotations

import os
import re

import pytest

import ouster_env as oe
from contract_run import VENDOR_PARAMS, run_main

BASE = {"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "os1-128", "SENSOR_IP": "192.168.1.201"}


def env(**extra: str) -> dict[str, str]:
    out = dict(BASE)
    out.update(extra)
    return out


def refused(e: dict[str, str]) -> list[str]:
    with pytest.raises(oe.Refused) as info:
        oe.parse(e)
    return info.value.problems


def render(e: dict[str, str]) -> str:
    return oe.render(oe.parse(e), str(VENDOR_PARAMS))


def params(text: str) -> dict[str, str]:
    """key -> value text of the rendered file's parameter lines."""
    out = {}
    for line in text.splitlines():
        m = re.match(r"^    ([a-z_]+): (.*)$", line)
        if m:
            assert m[1] not in out, f"{m[1]} twice"
            out[m[1]] = m[2]
    return out


# --- defaults -------------------------------------------------------------------------------

def test_defaults():
    s = oe.parse(BASE)
    assert (s.model.label, s.sensor_ip, s.data_port, s.imu_port) == ("OS1-128", "192.168.1.201", 7502, 7503)
    assert (s.namespace, s.node_fqn, s.topic_fqn) == ("/ouster", "/ouster/os_driver", "/ouster/points")
    assert (s.frame_id, s.sensor_frame, s.imu_frame) == ("os_lidar", "os_sensor", "os_imu")
    assert s.time_mode == "host" and s.host_ip is None
    assert s.params_path == "/tmp/ouster/driver_params.yaml"


def test_sensor_id_prefixes_namespace_and_every_frame():
    s = oe.parse(env(SENSOR_ID="east"))
    assert (s.node_fqn, s.topic_fqn) == ("/east/os_driver", "/east/points")
    assert (s.frame_id, s.sensor_frame, s.imu_frame) == ("east_os_lidar", "east_os_sensor", "east_os_imu")


def test_frame_id_sets_lidar_and_point_cloud_frame():
    p = params(render(env(FRAME_ID="mast_lidar")))
    assert p["lidar_frame"] == p["point_cloud_frame"] == '"mast_lidar"'
    assert p["sensor_frame"] == '"os_sensor"'


@pytest.mark.parametrize("value,expected", [
    ("host", "TIME_FROM_ROS_TIME"), ("HOST", "TIME_FROM_ROS_TIME"),
    ("sensor", "TIME_FROM_INTERNAL_OSC"), ("gps", "TIME_FROM_SYNC_PULSE_IN"),
    ("Ptp", "TIME_FROM_PTP_1588"),
])
def test_time_modes(value, expected):
    assert params(render(env(TIME_MODE=value)))["timestamp_mode"] == f'"{expected}"'


def test_host_ip_becomes_udp_dest_without_prefix():
    p = params(render(env(HOST_IP="192.168.1.100/24")))
    assert p["udp_dest"] == '"192.168.1.100"'
    assert params(render(BASE))["udp_dest"] == '""'


def test_ports():
    p = params(render(env(DATA_PORT="7600")))
    assert (p["lidar_port"], p["imu_port"]) == ("7600", "7601")
    assert oe.parse(env(DATA_PORT="65534")).imu_port == 65535


@pytest.mark.parametrize("name", ["os0", "os1", "os2", "osdome", "OS-0-32", "os1_64", "OS2-128",
                                  "OS-DOME-128", "ouster-os1-128", "Ouster_OSDome-64"])
def test_accepted_models(name):
    assert isinstance(oe.lookup_model(name), oe.Model)


@pytest.mark.parametrize("name", ["os1-16", "os3", "os1-128-u0", "os1 128", "os1.128", "vlp16",
                                  "ouster", "osdome-16", "hesai-xt32", "os-dome-128-x"])
def test_refused_models(name):
    assert isinstance(oe.lookup_model(name), str)


def test_model_names_cover_the_table():
    assert len(oe.MODELS) == 16
    for key, model in oe.MODELS.items():
        assert oe.lookup_model(model.label) is model
        assert key == oe.normalise_model(model.label)


# --- refusals -------------------------------------------------------------------------------

@pytest.mark.parametrize("switch", ["2", "yes", "true", " ", "01"])
def test_switch_values_other_than_1_are_refused(switch):
    problems = refused(env(METROLLA_DRIVER_CONTRACT=switch))
    assert len(problems) == 1 and problems[0].startswith("METROLLA_DRIVER_CONTRACT=")


def test_every_problem_is_reported_once():
    problems = refused({"METROLLA_DRIVER_CONTRACT": "1", "SENSOR_IP": "x", "DATA_PORT": "1",
                        "TIME_MODE": "ntp", "SENSOR_PORT": "2368", "LIDAR_MODLE": "os1"})
    assert [p.split("=")[0].split(" ")[0] for p in problems] == [
        "LIDAR_MODEL", "SENSOR_IP", "DATA_PORT", "TIME_MODE", "LIDAR_MODLE", "SENSOR_PORT"]


@pytest.mark.parametrize("value", ["0.0.0.0", "224.0.0.1", "240.0.0.1", "255.255.255.255",
                                   "192.168.001.201", "192.168.1", "os-992210000957.local", "::1"])
def test_bad_sensor_ip(value):
    assert refused(env(SENSOR_IP=value))[0].startswith("SENSOR_IP=")


@pytest.mark.parametrize("value", ["0", "80", "1023", "65535", "07502", "7502x", "-1"])
def test_bad_data_port(value):
    assert refused(env(DATA_PORT=value))[0].startswith("DATA_PORT=")


@pytest.mark.parametrize("value", ["east", "a", "east_2", "s" * 31])
def test_good_sensor_id(value):
    assert oe.parse(env(SENSOR_ID=value)).sensor_id == value


@pytest.mark.parametrize("value", ["East", "1east", "east__2", "east-2", "s" * 32, "_east"])
def test_bad_sensor_id(value):
    assert refused(env(SENSOR_ID=value))[0].startswith("SENSOR_ID=")


@pytest.mark.parametrize("value", ["points", "/points", "lidar/points", "/a/b_c/d1"])
def test_good_topics(value):
    assert oe.parse(env(POINTCLOUD_TOPIC=value)).topic == value


@pytest.mark.parametrize("value", ["/", "points/", "//points", "1points", "po ints", "a/__b", "~/points"])
def test_bad_topics(value):
    assert refused(env(POINTCLOUD_TOPIC=value))[0].startswith("POINTCLOUD_TOPIC=")


def test_device_port_only_80():
    assert oe.parse(env(DEVICE_PORT="80"))
    assert refused(env(DEVICE_PORT="7501"))[0].startswith("DEVICE_PORT=")


@pytest.mark.parametrize("name", ["PUBLISH_SCAN", "DUAL_LIDAR"])
@pytest.mark.parametrize("value", ["0", "false", "No", "OFF"])
def test_false_only_flags_accept_false(name, value):
    oe.parse(env(**{name: value}))


@pytest.mark.parametrize("name", ["PUBLISH_SCAN", "DUAL_LIDAR"])
def test_false_only_flags_refuse_true(name):
    assert refused(env(**{name: "true"}))[0].startswith(f"{name}=")


def test_empty_dual_lidar_means_false():
    oe.parse(env(DUAL_LIDAR=""))


def test_unrelated_variables_pass():
    oe.parse(env(ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp", HOSTNAME="x",
                 PATH="/usr/bin", ROS_DOMAIN_ID="3", USE_MACADDRESS_NS="0"))


@pytest.mark.parametrize("name,hint", [("SENSOR_MODEL", "LIDAR_MODEL"), ("POINT_CLOUD_TOPIC", "POINTCLOUD_TOPIC"),
                                       ("COMPUTER_IP", "HOST_IP"), ("OUSTER_RUNDIR", "OUSTER_RUN_DIR")])
def test_typos_name_the_contract_variable(name, hint):
    assert f"did you mean {hint}?" in refused(env(**{name: "x"}))[0]


@pytest.mark.parametrize("name", sorted(oe.LEGACY))
def test_legacy_names_are_refused(name):
    problem = refused(env(**{name: "x"}))[0]
    assert problem.startswith(f"{name} is a legacy name")
    if oe.LEGACY[name]:
        assert f"use {oe.LEGACY[name]}" in problem


def test_shown_values_are_safe():
    assert oe.shown("a\nb") == "'a\\nb'"
    assert oe.shown("x" * 100).endswith("...'")


# --- rendering ------------------------------------------------------------------------------

def test_render_sets_every_contract_key_once_and_keeps_vendor_defaults():
    text = render(BASE)
    p = params(text)
    vendor = params(VENDOR_PARAMS.read_text())
    wanted = oe.contract_params(oe.parse(BASE))
    for key, value in wanted.items():
        assert p[key] == oe.yaml_scalar(value), key
    for key in vendor:
        if key not in wanted:
            assert p[key] == vendor[key], f"{key} changed"
    assert p["auto_start"] == "true"
    assert set(p) == set(vendor) | {"auto_start"}
    assert text.count("/**:") == 1 and "ouster/os_driver:" not in text


def test_contract_settings_are_the_documented_ones():
    p = params(render(BASE))
    assert p["proc_mask"] == '"PCL|IMU|TLM"'
    assert p["persist_config"] == "false"
    assert p["attempt_reconnect"] == "true"
    assert p["mtp_dest"] == '""' and p["mtp_main"] == "false"
    assert p["metadata"] == '"/tmp/ouster/metadata.json"'


def test_render_refuses_a_vendor_file_without_a_key():
    source = VENDOR_PARAMS.read_text().replace("    imu_port: 0\n", "")
    with pytest.raises(oe.BrokenImage, match="imu_port"):
        oe.render_params(oe.parse(BASE), source)


def test_render_refuses_a_vendor_file_with_a_key_twice():
    source = VENDOR_PARAMS.read_text().replace("    imu_port: 0\n", "    imu_port: 0\n    imu_port: 1\n")
    with pytest.raises(oe.BrokenImage, match="imu_port"):
        oe.render_params(oe.parse(BASE), source)


def test_render_refuses_another_node_key():
    source = VENDOR_PARAMS.read_text().replace("ouster/os_driver:", "ouster/os_sensor:")
    with pytest.raises(oe.BrokenImage):
        oe.render_params(oe.parse(BASE), source)


def test_yaml_strings_are_quoted():
    assert oe.yaml_scalar("a: b") == '"a: b"'
    assert oe.yaml_scalar(True) == "true" and oe.yaml_scalar(5.0) == "5.0"


def test_node_argv():
    assert oe.node_argv(oe.parse(BASE)) == [
        oe.DRIVER_NODE, "--ros-args", "-r", "__ns:=/ouster", "-r", "__node:=os_driver",
        "--params-file", "/tmp/ouster/driver_params.yaml"]
    argv = oe.node_argv(oe.parse(env(SENSOR_ID="east", POINTCLOUD_TOPIC="/lidar/points")))
    assert argv[3:6] == ["__ns:=/east", "-r", "__node:=os_driver"]
    assert argv[-2:] == ["-r", "points:=/lidar/points"]


# --- command line ---------------------------------------------------------------------------

def test_check_prints_render_and_exec(monkeypatch):
    code, out, err = run_main(["check"], BASE, monkeypatch)
    assert code == 0 and err == ""
    assert out.splitlines()[-1].startswith("# exec: " + oe.DRIVER_NODE)


def test_check_json(monkeypatch):
    import json
    code, out, _ = run_main(["check", "--json"], env(SENSOR_ID="east"), monkeypatch)
    data = json.loads(out)
    assert code == 0 and data["pointcloud_topic"] == "/east/points" and data["imu_port"] == 7503
    assert data["sources"]["SENSOR_ID"] == "env" and data["sources"]["DATA_PORT"] == "default"


def test_check_json_refused(monkeypatch):
    import json
    code, out, err = run_main(["check", "--json"], {"METROLLA_DRIVER_CONTRACT": "1"}, monkeypatch)
    assert code == 1 and len(json.loads(out)["refused"]) == 2 and err.count("\n") == 2


@pytest.mark.parametrize("argv", [["bash"], ["ros2", "launch", "ouster_ros", "sensor.launch.xml"],
                                  ["check", "--yaml"], ["run", "x"]])
def test_unknown_commands_refused(argv, monkeypatch):
    code, _, err = run_main(argv, BASE, monkeypatch)
    assert code == 1 and "unknown command" in err


def test_help(monkeypatch):
    code, out, _ = run_main(["--help"], BASE, monkeypatch)
    assert code == 0 and out.startswith("usage:")


@pytest.mark.parametrize("argv", [[], ["run"], list(oe.LEGACY_DEFAULT_CMD)])
def test_run_writes_parameters_and_execs_the_node(argv, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(oe.os, "execve", lambda *a: calls.append(a) or (_ for _ in ()).throw(SystemExit(0)))
    e = env(OUSTER_RUN_DIR=str(tmp_path / "run"))
    e.pop("ROS_HOME", None)
    with pytest.raises(SystemExit):
        run_main(argv, e, monkeypatch)
    path, args, child_env = calls[0]
    assert path == oe.DRIVER_NODE and args[-1] == str(tmp_path / "run/driver_params.yaml")
    assert (tmp_path / "run/driver_params.yaml").read_text() == render(e)
    assert child_env["ROS_HOME"] == "/tmp/ros" and child_env["ROS_LOG_DIR"] == "/tmp/ros/log"
    assert oct((tmp_path / "run").stat().st_mode & 0o777) == "0o700"


def test_run_refused_does_not_exec(monkeypatch):
    monkeypatch.setattr(oe.os, "execve", lambda *a: pytest.fail("exec'd"))
    code, out, err = run_main([], {"METROLLA_DRIVER_CONTRACT": "1"}, monkeypatch)
    assert code == 1 and out == "" and err.count("refusing to start") == 2


def test_run_without_writable_dir_is_refused(monkeypatch, tmp_path):
    monkeypatch.setattr(oe.os, "execve", lambda *a: pytest.fail("exec'd"))
    blocked = tmp_path / "ro"
    blocked.mkdir()
    blocked.chmod(0o500)
    if os.access(blocked, os.W_OK):  # running as root
        pytest.skip("root can write anywhere")
    code, _, err = run_main([], env(OUSTER_RUN_DIR=str(blocked / "x")), monkeypatch)
    assert code == 1 and "cannot write" in err and "--tmpfs /tmp" in err


def test_run_broken_image(monkeypatch, tmp_path):
    monkeypatch.setattr(oe, "VENDOR_PARAMS", str(tmp_path / "missing.yaml"))
    import contextlib
    import io
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        assert oe.main([], BASE) == 1
    assert "broken image" in err.getvalue()


def test_selfcheck(tmp_path, capsys):
    node = tmp_path / "os_driver"
    node.write_text("")
    node.chmod(0o755)
    assert oe.cmd_selfcheck(str(VENDOR_PARAMS), str(node)) == 0
    assert oe.cmd_selfcheck(str(VENDOR_PARAMS), str(tmp_path / "missing")) == 1
    assert "driver binary missing" in capsys.readouterr().err


def test_legacy_default_cmd_is_the_dockerfile_cmd():
    import json
    from contract_run import REPO
    line = [l for l in (REPO / "Dockerfile.jazzy").read_text().splitlines() if l.startswith("CMD ")]
    assert len(line) == 1 and json.loads(line[0][4:]) == oe.LEGACY_DEFAULT_CMD


def test_image_paths_match_the_dockerfile():
    from contract_run import REPO
    text = (REPO / "Dockerfile.jazzy").read_text()
    assert "ENV ROS2_WS=/opt/ros2_ws" in text and oe.WORKSPACE == "/opt/ros2_ws"
    assert "COPY docker/contract-entrypoint.sh docker/ouster_env.py /opt/metrolla/ouster/" in text
    assert "/opt/metrolla/ouster/contract-entrypoint.sh" in (REPO / "entrypoint.sh").read_text()
    assert "/opt/metrolla/ouster/ouster_env.py" in (REPO / "docker/contract-entrypoint.sh").read_text()

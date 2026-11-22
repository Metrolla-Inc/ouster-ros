"""The smoke test's fake sensor and recording (docker/smoke): the recording is the pinned SDK test
data, the fake sensor answers the HTTP calls the Ouster SDK makes when it configures a sensor,
and the UDP data it sends is the recording with only the frame id and timestamps changed."""

from __future__ import annotations

import hashlib
import json
import socket
import struct
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

import ouster_packets as op

FAKE = Path(op.__file__).with_name("fake_sensor.py")


def test_recording_is_the_pinned_sdk_test_data():
    for suffix, digest in op.RECORDING_SHA256.items():
        data = Path(str(op.RECORDING) + suffix).read_bytes()
        assert hashlib.sha256(data).hexdigest() == digest, suffix


def test_recording_is_one_full_frame():
    rec = op.load()
    assert (rec.width, rec.height, rec.columns_per_packet) == (1024, 128, 16)
    assert len(rec.lidar) == 64 and len(rec.imu) == 10
    mids = [struct.unpack_from("<HH", p, off + 8) for p in rec.lidar for off in op.column_offsets(rec)]
    assert [m for m, _ in mids] == list(range(1024)) and all(s & 1 for _, s in mids)
    assert len({op.frame_id(p) for p in rec.lidar}) == 1
    init_id = struct.unpack_from("<I", rec.lidar[0], 4)[0] & 0xFFFFFF
    assert init_id == rec.meta["initialization_id"]


def test_nested_metadata_has_the_sensor_api_shape():
    meta = op.nested_metadata(op.load(), "10.0.0.2", 7600, 7601, "TIME_FROM_PTP_1588")
    assert set(meta) == {"sensor_info", "beam_intrinsics", "imu_intrinsics", "lidar_intrinsics",
                         "lidar_data_format", "calibration_status", "config_params"}
    assert meta["sensor_info"]["status"] == "RUNNING" and meta["sensor_info"]["prod_line"] == "OS-2-128"
    assert len(meta["beam_intrinsics"]["beam_altitude_angles"]) == 128
    cfg = meta["config_params"]
    assert (cfg["udp_dest"], cfg["udp_port_lidar"], cfg["udp_port_imu"]) == ("10.0.0.2", 7600, 7601)
    assert cfg["udp_profile_lidar"] == meta["lidar_data_format"]["udp_profile_lidar"] == op.PROFILE


def test_restamp_changes_only_frame_id_and_timestamps():
    rec = op.load()
    first = op.first_timestamp(rec)
    p = rec.lidar[3]
    q = op.restamp_lidar(rec, p, 7, 10**18, first)
    assert len(q) == len(p) and op.frame_id(q) == 7
    diff = [i for i in range(len(p)) if p[i] != q[i]]
    allowed = {2, 3} | {off + k for off in op.column_offsets(rec) for k in range(8)}
    assert set(diff) <= allowed
    for off in op.column_offsets(rec):
        assert struct.unpack_from("<Q", q, off)[0] == 10**18 + struct.unpack_from("<Q", p, off)[0] - first


def test_destagger_conventions():
    img = [[1, 2, 3, 4], [5, 6, 7, 8]]
    assert op.destagger(img, [1, 0], 1) == [[4, 1, 2, 3], [5, 6, 7, 8]]
    assert op.destagger(img, [1, 0], -1) == [[2, 3, 4, 1], [5, 6, 7, 8]]


def _free_port(kind=socket.SOCK_STREAM) -> int:
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def fake(tmp_path):
    port = _free_port()
    summary = tmp_path / "summary.json"
    proc = subprocess.Popen([sys.executable, str(FAKE), "--sensor-ip", "127.0.0.1", "--http-port",
                             str(port), "--seconds", "8", "--summary", str(summary)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    base = f"http://127.0.0.1:{port}/"
    for _ in range(100):
        try:
            urllib.request.urlopen(base + "api/v1/system/firmware", timeout=1).read()
            break
        except OSError:
            time.sleep(0.05)
    yield base, proc, summary
    proc.terminate()
    proc.wait(timeout=10)


def get(base: str, path: str) -> str:
    return urllib.request.urlopen(base + path, timeout=5).read().decode()


def test_fake_sensor_follows_the_sdk_configuration_sequence(fake):
    base, proc, summary = fake
    assert "v2.3.0" in json.loads(get(base, "api/v1/system/firmware"))["fw"]
    assert json.loads(get(base, "api/v1/sensor/metadata/sensor_info"))["status"] == "RUNNING"
    active = json.loads(get(base, "api/v1/sensor/cmd/get_config_param?args=active"))
    lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    lidar.bind(("127.0.0.1", 0))
    lidar.settimeout(5)
    lport = lidar.getsockname()[1]
    # What ouster::sensor::set_config sends: the whole active config with the changes, as JSON,
    # URL-encoded, after "args=.+"; then set_udp_dest_auto and reinitialize.
    active.update({"udp_port_lidar": lport, "udp_port_imu": 0, "timestamp_mode": "TIME_FROM_PTP_1588"})
    blob = urllib.parse.quote(json.dumps(active, separators=(",", ":")), safe="")
    assert get(base, "api/v1/sensor/cmd/set_config_param?args=.+" + blob) == '"set_config_param"'
    assert get(base, "api/v1/sensor/cmd/set_udp_dest_auto") == "{}"
    staged = json.loads(get(base, "api/v1/sensor/cmd/get_config_param?args=staged"))
    assert staged["udp_dest"] == "127.0.0.1"
    assert get(base, "api/v1/sensor/cmd/reinitialize") == "{}"
    meta = json.loads(get(base, "api/v1/sensor/metadata"))
    assert meta["config_params"]["udp_port_lidar"] == lport
    assert meta["config_params"]["timestamp_mode"] == "TIME_FROM_PTP_1588"

    rec = op.load()
    frames: dict[int, list[bytes]] = {}
    start = time.time_ns()
    while sum(len(v) for v in frames.values()) < 3 * 64:
        data = lidar.recv(65536)
        frames.setdefault(op.frame_id(data), []).append(data)
    ids = sorted(frames)
    full = [i for i in ids if len(frames[i]) == 64]
    assert full, f"no complete frame: {[(i, len(frames[i])) for i in ids]}"
    packets = frames[full[0]]
    for got, recorded in zip(packets, rec.lidar):
        assert len(got) == len(recorded)
        for off in op.column_offsets(rec):
            body = slice(off + 8, off + op.COLUMN_HEADER + rec.height * op.PIXEL_BYTES)
            assert got[body] == recorded[body]
            ts = struct.unpack_from("<Q", got, off)[0]
            assert abs(ts - start) < 5 * 10**9
    proc.terminate()
    proc.wait(timeout=10)
    data = json.loads(summary.read_text())
    assert data["reinits"] == 1 and data["persisted"] == 0 and data["sent"]["lidar"] >= 64


def test_fake_sensor_unknown_paths_are_404(fake):
    base, _, _ = fake
    with pytest.raises(urllib.error.HTTPError) as info:
        get(base, "api/v1/nothing")
    assert info.value.code == 404


import urllib.error  # noqa: E402
import urllib.parse  # noqa: E402

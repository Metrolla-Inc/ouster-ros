"""The smoke test's cloud check (docker/smoke/check_cloud.py) against clouds built here from the
recording: a correct cloud passes, a cloud with one wrong pixel, a wrong distance or no
destaggering does not."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

import check_cloud as cc
import ouster_packets as op


@dataclass
class Field:
    name: str
    offset: int
    datatype: int
    count: int = 1


@dataclass
class Cloud:
    width: int
    height: int
    point_step: int
    data: bytes
    fields: list = field(default_factory=list)
    is_bigendian: bool = False


# ouster_ros::Point ("original"): x y z (pad) intensity t reflectivity ring ambient range
LAYOUT = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 16, 7), ("t", 20, 6),
          ("reflectivity", 24, 4), ("ring", 26, 4), ("ambient", 28, 4), ("range", 32, 6)]
STEP = 48


def build(rec, sign=1, tamper=None, distance_error_mm=0.0, times=None):
    times = times or [0] * (rec.width * rec.height)
    stag = op.staggered_image(rec)
    img = {c: op.destagger(stag[c], rec.shifts, sign) if sign else stag[c] for c in stag}
    h, w = rec.height, rec.width
    alt = rec.meta["beam_altitude_angles"]
    buf = bytearray(h * w * STEP)
    for u in range(h):
        for v in range(w):
            i = u * w + v
            r = img["range"][u][v]
            d = (r + distance_error_mm) / 1000.0
            az = 2 * math.pi * v / w
            el = math.radians(alt[u])
            xyz = (d * math.cos(el) * math.cos(az), d * math.cos(el) * math.sin(az), d * math.sin(el))
            vals = {"x": xyz[0], "y": xyz[1], "z": xyz[2], "intensity": float(img["signal"][u][v]),
                    "t": times[i], "reflectivity": img["reflectivity"][u][v], "ring": u,
                    "ambient": img["near_ir"][u][v], "range": r}
            if tamper and tamper[0] == i:
                vals[tamper[1]] = tamper[2]
            for name, off, dt in LAYOUT:
                struct.pack_into("<" + cc.FIELD_FORMATS[dt], buf, i * STEP + off, vals[name])
    return Cloud(w, h, STEP, bytes(buf), [Field(n, o, d) for n, o, d in LAYOUT])


REC = op.load()
GOOD = build(REC)


def test_a_correct_cloud_passes():
    report = cc.compare(GOOD, REC)
    assert report["problems"] == [] and report["destagger_sign"] == 1
    assert report["valid_returns"] > 100000 and report["distance_checked"] > 100000


def test_one_wrong_pixel_fails():
    report = cc.compare(build(REC, tamper=(5000, "reflectivity", 255)), REC)
    assert report["problems"] == ["reflectivity_equal: 131071 of 131072 pixels"]


def test_wrong_ring_fails():
    assert cc.compare(build(REC, tamper=(7, "ring", 3)), REC)["problems"]


def test_distance_off_by_more_than_the_beam_offset_fails():
    report = cc.compare(build(REC, distance_error_mm=20.0), REC)
    assert any("distance" in p for p in report["problems"])


def test_other_destagger_direction_is_detected_not_failed():
    report = cc.compare(build(REC, sign=-1), REC)
    assert report["destagger_sign"] == -1 and report["problems"] == []


def test_missing_field_is_reported():
    cloud = build(REC)
    cloud.fields = [f for f in cloud.fields if f.name != "ambient"]
    try:
        cc.compare(cloud, REC)
    except ValueError as exc:
        assert "ambient" in str(exc)
    else:
        raise AssertionError("no error")


def test_a_cloud_that_is_not_destaggered_fails():
    report = cc.compare(build(REC, sign=0), REC)
    assert report["destagger_sign"] == 0 and any("destaggered" in p for p in report["problems"])


def test_point_times_are_counted_and_checked_on_request():
    new_rule = op.point_times(REC, 1)
    old_rule = op.point_times(REC, 1, measured_column=False)
    n = REC.width * REC.height
    report = cc.compare(build(REC, times=new_rule), REC, check_t=True)
    assert report["problems"] == [] and report["t_measured_column"] == n
    report = cc.compare(build(REC, times=old_rule), REC, check_t=True)
    assert report["t_output_column"] == n and report["t_measured_column"] == n // 4
    assert any(p.startswith("t: ") for p in report["problems"])
    # without --check-t the released image's rule is only counted (the legacy comparison)
    assert cc.compare(build(REC, times=old_rule), REC)["problems"] == []


@dataclass
class Vec:
    x: float
    y: float
    z: float


@dataclass
class Stamp:
    sec: int
    nanosec: int


@dataclass
class Header:
    stamp: Stamp
    frame_id: str


@dataclass
class ImuMsg:
    header: Header
    linear_acceleration: Vec
    angular_velocity: Vec


def imu_msg(values, frame="os_imu", stamp=1000.0):
    return ImuMsg(Header(Stamp(int(stamp), int((stamp % 1) * 1e9)), frame),
                  Vec(*values[:3]), Vec(*values[3:]))


def test_imu_values_must_be_a_recorded_packets_in_si_units():
    expected = cc.imu_expected(REC)
    assert abs(expected[0][2] - op.imu_reading(REC.imu[0])[2] * 9.80665) < 1e-9
    msgs = [(1000.0, imu_msg(e)) for e in expected[:3]]
    assert cc.check_imu(msgs, REC, "os_imu", 5.0) == {"messages": 3, "matched": 3, "problems": []}
    raw = op.imu_reading(REC.imu[0])  # g and deg/s, not converted
    bad = cc.check_imu([(1000.0, imu_msg(raw))], REC, "os_imu", 5.0)
    assert bad["matched"] == 0 and "none of the recorded" in bad["problems"][0]


def test_imu_frame_and_stamp_are_checked():
    e = cc.imu_expected(REC)[0]
    report = cc.check_imu([(1000.0, imu_msg(e, frame="east_os_imu", stamp=1010.0))], REC,
                          "os_imu", 5.0)
    assert len(report["problems"]) == 2

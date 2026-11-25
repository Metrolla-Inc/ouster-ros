"""The smoke test's cloud check (docker/smoke/check_cloud.py) against clouds built here from the
recording: a correct cloud passes, a cloud with one wrong pixel, a wrong distance or no
destaggering does not."""

from __future__ import annotations

from dataclasses import dataclass

import check_cloud as cc
import ouster_packets as op
from clouds import build


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


# Start-up scans (check_cloud.compare allow_partial, compare_all)

FIRST_PACKETS = range(0, 17 * 16)  # what a full socket buffer kept: the first 17 packets
TIMES = op.point_times(REC, 1)


def test_a_start_up_scan_is_accepted_only_as_the_first_cloud():
    partial = build(REC, times=TIMES, missing=set(range(REC.width)) - set(FIRST_PACKETS))
    report = cc.compare(partial, REC, check_t=True, allow_partial=True)
    assert report["problems"] == [] and report["partial"] is True
    assert report["columns_present"] == 272 and report["columns_empty"] == 752
    assert cc.compare(partial, REC, check_t=True)["problems"]  # not accepted when strict


def test_a_start_up_scan_with_a_wrong_pixel_in_a_present_column_fails():
    partial = build(REC, times=TIMES, missing=range(300, 1024), tamper=(100, "reflectivity", 255))
    report = cc.compare(partial, REC, check_t=True, allow_partial=True)
    assert len(report["problems"]) == 1 and "neither equal" in report["problems"][0]


def test_a_start_up_scan_with_a_wrong_time_in_a_present_column_fails():
    times = list(TIMES)
    times[100] += 1  # row 0, output column 100: a present measured column
    partial = build(REC, times=times, missing=range(300, 1024))
    assert cc.compare(partial, REC, check_t=True, allow_partial=True)["problems"]
    assert cc.compare(partial, REC, check_t=False, allow_partial=True)["problems"] == []


def test_a_start_up_scan_with_nothing_present_fails():
    empty = build(REC, times=TIMES, missing=range(REC.width))
    report = cc.compare(empty, REC, check_t=True, allow_partial=True)
    assert any("no column equal" in p for p in report["problems"])


def test_a_complete_first_cloud_is_not_reported_partial():
    report = cc.compare(build(REC, times=TIMES), REC, check_t=True, allow_partial=True)
    assert report["problems"] == [] and "partial" not in report


def test_every_cloud_after_the_first_must_be_complete():
    good = build(REC, times=TIMES)
    partial = build(REC, times=TIMES, missing=range(272, 1024))
    assert cc.compare_all([partial, good, good], REC, True)["problems"] == []
    out = cc.compare_all([good, good, partial], REC, True)
    assert out["problems"] and all(p.startswith("pixels (cloud 3): ") for p in out["problems"])
    assert out["later_clouds"][0]["cloud"] == 3
    out = cc.compare_all([good, partial, good], REC, True)
    assert out["problems"] and all(p.startswith("pixels (cloud 2): ") for p in out["problems"])
    assert out["pixels"]["range_equal"] < REC.width * REC.height

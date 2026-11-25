"""Past bug in the smoke test's cloud check (docker/smoke/check_cloud.py), D3e, 25 Nov 2026.

ros2 a13960b failed smoke step 2 (CI runs 45448862041 and 45566950314, two attempts each, every
time the same numbers): "legacy: cloud check failed", range_equal 43625, reflectivity_equal 41319,
intensity_equal and ambient_equal 34816 of 131072 pixels, t_measured_column 34816,
t_output_column 8800. Those are exactly the numbers of a scan holding only the first 17 lidar
packets of the recording (measurement ids 0-271; 17 x 24832 bytes is what a socket buffer of 2 x
212992 bytes, the kernel's default rmem_max, holds), every other column zero: the driver's
start-up scan, which the vendor publishes (min_scan_valid_columns_ratio 0.0, as in v1.0.0). The
check compared only the first cloud it received and required it complete, so it failed whenever
its subscription was matched before the driver's first scan.
"""

from __future__ import annotations

import check_cloud as cc
import ouster_packets as op
from clouds import build

REC = op.load()
N = REC.width * REC.height
TIMES = op.point_times(REC, 1)
START_UP = build(REC, times=TIMES, missing=range(17 * 16, REC.width))
FULL = build(REC, times=TIMES)


def test_the_ci_numbers_are_a_scan_of_the_first_17_packets():
    report = cc.compare(START_UP, REC, check_t=True)
    assert (report["range_equal"], report["reflectivity_equal"], report["intensity_equal"],
            report["ambient_equal"], report["t_measured_column"], report["t_output_column"]) \
        == (43625, 41319, 34816, 34816, 34816, 8800)


# PR #9's first CI run (45577055001) then showed the second start-up scan: the last 28 packets of
# a later frame (measurement ids 576-1023), its point times relative to a scan start the driver
# imputes across the frames it missed; the third cloud was complete and exact.
SECOND = build(REC, times=[t + 5_000_000 for t in TIMES], missing=range(0, REC.width - 28 * 16))


def test_the_second_ci_numbers_are_the_last_28_packets_of_a_frame():
    report = cc.compare(SECOND, REC)
    assert (report["range_equal"], report["reflectivity_equal"], report["intensity_equal"]) \
        == (65641, 64188, 57344)


def test_start_up_scans_then_complete_clouds_pass():
    out = cc.compare_all([START_UP, SECOND, FULL, FULL, FULL], REC, True)
    assert out["problems"] == []
    assert [r["columns_present"] for r in out["start_up_clouds"]] == [272, 448]
    assert out["pixels"]["range_equal"] == N and out["pixels"]["t_measured_column"] == N


def test_a_start_up_scan_after_a_complete_cloud_still_fails():
    out = cc.compare_all([FULL, START_UP, FULL], REC, True)
    assert "pixels (cloud 2): range_equal: 43625 of 131072 pixels" in out["problems"]

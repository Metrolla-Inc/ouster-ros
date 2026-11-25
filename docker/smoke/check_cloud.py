#!/usr/bin/env python3
"""Smoke check, run inside the driver image: subscribe to a PointCloud2 topic and check what the
driver published from the fake sensor's recorded packets (fake_sensor.py).

With --imu-topic, sensor_msgs/Imu too: frame, stamps, and every message's acceleration (m/s^2)
and angular velocity (rad/s) equal to one of the recorded IMU packets' (g and deg/s converted).

For each cloud: frame id, organized size (pixels_per_column x columns_per_frame of the recording),
stamp within --stamp-window seconds of this machine's clock and increasing. Every cloud, pixel by
pixel against the recording: the range, reflectivity, signal (intensity) and near-IR (ambient)
fields equal the recorded packets' values after destaggering, ring is the row, and the point's
distance from the origin matches its range (within the beam-origin offset). The per-point time
``t`` is counted against both rules (ouster_packets.point_times): the time of the column the
point was measured in (ouster-ros 0.14+) and the time of its output column (the v1.0.0 base);
with --check-t every point must follow the first.

The first cloud received may be the driver's start-up scan: the driver opens its UDP socket and
points the sensor at it before it starts reading, so the socket's buffer holds the first packets
of a frame and drops the rest, and the vendor's default min_scan_valid_columns_ratio (0.0)
publishes that scan with the missing columns zeroed (a real sensor gives the same partial scan
when the driver starts mid-rotation). So in the first cloud each measured column must either
equal the recording in every row or be empty (all four channels zero in every row), with at least
one column present; every later cloud must be complete and equal. ``pixels`` in the report is the
second cloud's comparison, ``first_cloud`` the first's.

Prints a JSON report; exit 0 when every check passed, 1 otherwise.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ouster_packets as op  # noqa: E402

# sensor_msgs/PointField datatypes
FIELD_FORMATS = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def decode(msg, names: list[str]) -> dict[str, list]:
    fields = {f.name: f for f in msg.fields}
    missing = [n for n in names if n not in fields]
    if missing:
        raise ValueError(f"cloud has no field(s) {missing}; fields: {sorted(fields)}")
    endian = ">" if msg.is_bigendian else "<"
    data = bytes(msg.data)
    n = msg.width * msg.height
    if len(data) < n * msg.point_step:
        raise ValueError(f"cloud data has {len(data)} bytes, expected {n * msg.point_step}")
    out = {}
    for name in names:
        f = fields[name]
        unpack = struct.Struct(endian + FIELD_FORMATS[f.datatype]).unpack_from
        step, off = msg.point_step, f.offset
        out[name] = [unpack(data, off + i * step)[0] for i in range(n)]
    return out


def compare(msg, rec: op.Recording, check_t: bool = False, allow_partial: bool = False) -> dict:
    """Pixel comparison of one cloud with the recording. allow_partial: the cloud may be a
    start-up scan with whole measured columns missing (all channels zero); the columns present
    must be exact."""
    h, w = rec.height, rec.width
    stag = op.staggered_image(rec)
    vals = decode(msg, ["x", "y", "z", "range", "reflectivity", "intensity", "ambient", "ring"])
    best = None
    for sign in (1, -1, 0):
        img = op.destagger(stag["range"], rec.shifts, sign) if sign else stag["range"]
        flat = [x for row in img for x in row]
        same = sum(1 for a, b in zip(flat, vals["range"]) if a == b)
        if best is None or same > best[1]:
            best = (sign, same)
    sign, same = best
    n = h * w

    def flat(channel):
        img = op.destagger(stag[channel], rec.shifts, sign) if sign else stag[channel]
        return [x for row in img for x in row]

    report = {"pixels": n, "destagger_sign": sign, "range_equal": same,
              "valid_returns": sum(1 for r in flat("range") if r > 0)}
    report["reflectivity_equal"] = sum(1 for a, b in zip(flat("reflectivity"), vals["reflectivity"])
                                       if a == b)
    report["intensity_equal"] = sum(1 for a, b in zip(flat("signal"), vals["intensity"])
                                    if a == float(b) or abs(a - b) < 0.5)
    report["ambient_equal"] = sum(1 for a, b in zip(flat("near_ir"), vals["ambient"]) if a == b)
    report["ring_is_row"] = sum(1 for i, r in enumerate(vals["ring"]) if r == i // w)
    if "t" in {f.name for f in msg.fields}:
        t = decode(msg, ["t"])["t"]
        report["t_measured_column"] = sum(1 for a, b in zip(op.point_times(rec, sign or 1), t)
                                          if a == b)
        report["t_output_column"] = sum(1 for a, b in zip(op.point_times(rec, sign or 1, False), t)
                                        if a == b)
    elif check_t:
        raise ValueError("cloud has no field 't'")
    worst = 0.0
    checked = 0
    for i, r in enumerate(vals["range"]):
        if r < 1000:
            continue
        x, y, z = vals["x"][i], vals["y"][i], vals["z"][i]
        if any(math.isnan(c) for c in (x, y, z)):
            worst = float("inf")
            break
        worst = max(worst, abs(math.sqrt(x * x + y * y + z * z) * 1000.0 - r))
        checked += 1
    report["distance_checked"] = checked
    report["distance_worst_mm"] = round(worst, 3)
    beam_offset = rec.meta["lidar_origin_to_beam_origin_mm"]
    report["problems"] = []
    complete = all(report[k] == n for k in ("range_equal", "reflectivity_equal",
                                            "intensity_equal", "ambient_equal"))
    t_ok = not check_t or report.get("t_measured_column") == n
    if allow_partial and not complete:
        report.update(partial_columns(vals, msg, rec, sign, flat, check_t))
        report["problems"] += report.pop("partial_problems")
    else:
        for key in ("range_equal", "reflectivity_equal", "intensity_equal", "ambient_equal"):
            if report[key] != n:
                report["problems"].append(f"{key}: {report[key]} of {n} pixels")
        if not t_ok:
            report["problems"].append(f"t: {report.get('t_measured_column')} of {n} points carry "
                                      "the time of the column they were measured in")
    if report["ring_is_row"] != n:
        report["problems"].append(f"ring_is_row: {report['ring_is_row']} of {n} pixels")
    if sign == 0:
        report["problems"].append("the cloud is in the sensor's column order, not destaggered "
                                  "(the driver's destagger parameter is true)")
    if report["valid_returns"] < n // 2:
        report["problems"].append(f"only {report['valid_returns']} returns in the recording?")
    if not checked or worst > beam_offset + 1.0:
        report["problems"].append(f"distance vs range: worst {worst:.3f} mm over {checked} points "
                                  f"(allowed {beam_offset + 1.0:.3f} mm)")
    return report


def partial_columns(vals: dict, msg, rec: op.Recording, sign: int, flat, check_t: bool) -> dict:
    """A start-up scan: per measured column (the sensor's column the pixel came from), every row
    equal to the recording in all four channels (and t with check_t: the measured column's time)
    or every row empty (range, reflectivity, intensity and ambient all zero)."""
    h, w = rec.height, rec.width
    ref = {"range": flat("range"), "reflectivity": flat("reflectivity"),
           "intensity": flat("signal"), "ambient": flat("near_ir")}
    times = op.point_times(rec, sign or 1) if check_t else None
    t = decode(msg, ["t"])["t"] if check_t else None
    equal = [True] * w
    empty = [True] * w
    for u in range(h):
        for v in range(w):
            i = u * w + v
            m = (v - sign * rec.shifts[u]) % w if sign else v
            if (vals["range"][i] != ref["range"][i]
                    or vals["reflectivity"][i] != ref["reflectivity"][i]
                    or abs(vals["intensity"][i] - ref["intensity"][i]) >= 0.5
                    or vals["ambient"][i] != ref["ambient"][i]
                    or (check_t and t[i] != times[i])):
                equal[m] = False
            if (vals["range"][i] or vals["reflectivity"][i] or vals["intensity"][i]
                    or vals["ambient"][i]):
                empty[m] = False
    present = sum(equal)
    bad = [m for m in range(w) if not equal[m] and not empty[m]]
    problems = []
    if bad:
        problems.append(f"start-up scan: {len(bad)} of {w} columns neither equal to the recording "
                        f"nor empty (first: measurement id {bad[0]})")
    if not present:
        problems.append("start-up scan: no column equal to the recording")
    return {"partial": True, "columns_present": present,
            "columns_empty": sum(1 for m in range(w) if empty[m] and not equal[m]),
            "partial_problems": problems}


def imu_expected(rec: op.Recording) -> list[tuple[float, ...]]:
    """(ax, ay, az, wx, wy, wz) of each recorded IMU packet in the units sensor_msgs/Imu uses."""
    return [tuple(v * op.STANDARD_G for v in op.imu_reading(p)[:3])
            + tuple(math.radians(v) for v in op.imu_reading(p)[3:]) for p in rec.imu]


def check_imu(msgs: list[tuple[float, object]], rec: op.Recording, frame: str,
              stamp_window: float) -> dict:
    """Each IMU message: frame, stamp near its receive time, and acceleration and angular
    velocity equal to one of the recorded packets' (float32 precision)."""
    expected = imu_expected(rec)
    report = {"messages": len(msgs), "matched": 0, "problems": []}
    for received, m in msgs:
        got = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
               m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        if any(all(abs(a - b) <= 1e-5 * max(1.0, abs(b)) for a, b in zip(got, e))
               for e in expected):
            report["matched"] += 1
        else:
            report["problems"].append(f"imu values {got} are none of the recorded packets'")
        if m.header.frame_id != frame:
            report["problems"].append(f"imu frame_id {m.header.frame_id!r}, expected {frame!r}")
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if abs(stamp - received) > stamp_window:
            report["problems"].append(f"imu stamp {stamp:.3f} is {stamp - received:+.3f} s from "
                                      "the receive time")
    report["problems"] = report["problems"][:10]
    return report


def compare_all(msgs: list, rec: op.Recording, check_t: bool) -> dict:
    """Pixel reports for every cloud: the first may be a start-up scan (compare allow_partial),
    the others must be complete. Returns first_cloud, pixels (the second cloud's report, or the
    first's when only one arrived), later_clouds (problems of the third and later) and problems."""
    out: dict = {"problems": [], "later_clouds": []}
    for k, msg in enumerate(msgs):
        try:
            rep = compare(msg, rec, check_t, allow_partial=(k == 0))
        except ValueError as exc:
            rep = {"problems": [str(exc)]}
        label = "pixels" if k == 1 else "first_cloud" if k == 0 else None
        if label:
            out[label] = rep
        else:
            out["later_clouds"].append({"cloud": k + 1, "problems": rep["problems"]})
        out["problems"] += [f"pixels (cloud {k + 1}): {p}" for p in rep["problems"]]
    if len(msgs) == 1:
        out["pixels"] = out["first_cloud"]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("frame")
    ap.add_argument("--clouds", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--stamp-window", type=float, default=5.0)
    ap.add_argument("--recording", default=str(op.RECORDING))
    ap.add_argument("--no-pixels", action="store_true", help="skip the pixel comparison")
    ap.add_argument("--check-t", action="store_true",
                    help="every point's t must be its measured column's time (ouster-ros 0.14+)")
    ap.add_argument("--imu-topic", default="", help="also check sensor_msgs/Imu on this topic")
    ap.add_argument("--imu-frame", default="os_imu")
    ap.add_argument("--imus", type=int, default=5)
    args = ap.parse_args()

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2

    rec = op.load(Path(args.recording))
    rclpy.init()
    node = rclpy.create_node("metrolla_smoke_check")
    got: list[tuple[float, object]] = []
    node.create_subscription(PointCloud2, args.topic, lambda m: got.append((time.time(), m)),
                             qos_profile_sensor_data)
    imus: list[tuple[float, object]] = []
    if args.imu_topic:
        node.create_subscription(Imu, args.imu_topic, lambda m: imus.append((time.time(), m)),
                                 qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while (len(got) < args.clouds or (args.imu_topic and len(imus) < args.imus)) \
            and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    report = {"topic": args.topic, "clouds": len(got), "problems": []}
    if len(got) < args.clouds:
        report["problems"].append(f"received {len(got)} of {args.clouds} clouds in {args.timeout} s")
    last = None
    clouds = []
    for received, m in got:
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        c = {"frame_id": m.header.frame_id, "width": m.width, "height": m.height,
             "points": m.width * m.height, "stamp_minus_receive_s": round(stamp - received, 3),
             "fields": [f.name for f in m.fields]}
        clouds.append(c)
        if m.header.frame_id != args.frame:
            report["problems"].append(f"frame_id {m.header.frame_id!r}, expected {args.frame!r}")
        if (m.height, m.width) != (rec.height, rec.width):
            report["problems"].append(f"cloud {m.height}x{m.width}, expected organized "
                                      f"{rec.height}x{rec.width}")
        if abs(stamp - received) > args.stamp_window:
            report["problems"].append(f"stamp {stamp:.3f} is {stamp - received:+.3f} s from the "
                                      "receive time")
        if last is not None and stamp <= last:
            report["problems"].append(f"stamp {stamp:.6f} not after the previous {last:.6f}")
        last = stamp
    report["cloud_list"] = clouds
    if got and not args.no_pixels:
        pixels = compare_all([m for _, m in got], rec, args.check_t)
        report["problems"] += pixels.pop("problems")
        report.update(pixels)
    if args.imu_topic:
        imu = check_imu(imus, rec, args.imu_frame, args.stamp_window)
        imu["topic"] = args.imu_topic
        if len(imus) < args.imus:
            imu["problems"].append(f"received {len(imus)} of {args.imus} IMU messages")
        report["imu"] = imu
        report["problems"] += [f"imu: {p}" for p in imu["problems"]]
    print(json.dumps(report, indent=2))
    return 1 if report["problems"] else 0


if __name__ == "__main__":
    sys.exit(main())

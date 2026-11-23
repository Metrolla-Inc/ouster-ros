#!/usr/bin/env python3
"""Smoke check, run inside the driver image: subscribe to a PointCloud2 topic and check what the
driver published from the fake sensor's recorded packets (fake_sensor.py).

With --imu-topic, sensor_msgs/Imu too: frame, stamps, and every message's acceleration (m/s^2)
and angular velocity (rad/s) equal to one of the recorded IMU packets' (g and deg/s converted).

For each cloud: frame id, organized size (pixels_per_column x columns_per_frame of the recording),
stamp within --stamp-window seconds of this machine's clock and increasing. For the first cloud,
pixel by pixel against the recording: the range, reflectivity, signal (intensity) and near-IR
(ambient) fields equal the recorded packets' values after destaggering, ring is the row, and the
point's distance from the origin matches its range (within the beam-origin offset). The per-point
time ``t`` is counted against both rules (ouster_packets.point_times): the time of the column
the point was measured in (ouster-ros 0.14+) and the time of its output column (the v1.0.0 base);
with --check-t every point must follow the first.

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


def compare(msg, rec: op.Recording, check_t: bool = False) -> dict:
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
    for key in ("range_equal", "reflectivity_equal", "intensity_equal", "ambient_equal",
                "ring_is_row"):
        if report[key] != n:
            report["problems"].append(f"{key}: {report[key]} of {n} pixels")
    if check_t and report.get("t_measured_column") != n:
        report["problems"].append(f"t: {report.get('t_measured_column')} of {n} points carry the "
                                  "time of the column they were measured in")
    if sign == 0:
        report["problems"].append("the cloud is in the sensor's column order, not destaggered "
                                  "(the driver's destagger parameter is true)")
    if report["valid_returns"] < n // 2:
        report["problems"].append(f"only {report['valid_returns']} returns in the recording?")
    if not checked or worst > beam_offset + 1.0:
        report["problems"].append(f"distance vs range: worst {worst:.3f} mm over {checked} points "
                                  f"(allowed {beam_offset + 1.0:.3f} mm)")
    return report


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
        try:
            pixels = compare(got[0][1], rec, args.check_t)
        except ValueError as exc:
            pixels = {"problems": [str(exc)]}
        report["pixels"] = pixels
        report["problems"] += [f"pixels: {p}" for p in pixels["problems"]]
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

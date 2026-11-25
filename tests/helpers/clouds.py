"""Organized PointCloud2-like clouds built from the smoke recording, for tests of the smoke
test's cloud check (docker/smoke/check_cloud.py): exact, tampered, not destaggered, or a start-up
scan with whole measured columns missing."""

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


def build(rec, sign=1, tamper=None, distance_error_mm=0.0, times=None, missing=()):
    """missing: measurement ids (sensor columns) absent from the scan, as the driver publishes
    them: every channel and t zero."""
    missing = set(missing)
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
            m = (v - sign * rec.shifts[u]) % w if sign else v
            if m in missing:
                vals.update(x=0.0, y=0.0, z=0.0, range=0, reflectivity=0, intensity=0.0,
                            ambient=0, t=0)
            if tamper and tamper[0] == i:
                vals[tamper[1]] = tamper[2]
            for name, off, dt in LAYOUT:
                struct.pack_into("<" + cc.FIELD_FORMATS[dt], buf, i * STEP + off, vals[name])
    return Cloud(w, h, STEP, bytes(buf), [Field(n, o, d) for n, o, d in LAYOUT])

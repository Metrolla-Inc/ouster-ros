"""Recorded Ouster packets for the smoke test: read them, turn them into what a sensor would send,
and say what the driver must publish from them.

The recording is the Ouster SDK's own test data, in this repository's SDK submodule:
ouster-ros/ouster-sdk/tests/pcaps/OS-2-128-U1_v2.3.0_1024x10.{pcap,json} - one full frame (64 lidar
packets of 16 columns, 1024x10, profile RNG19_RFL8_SIG16_NIR16) and 10 IMU packets from a real
OS-2-128 on firmware 2.3.0, with the sensor's metadata in the legacy (flat) JSON format.

A sensor's HTTP API serves metadata in the newer nested format (api/v1/sensor/metadata), so
``nested_metadata`` converts the recording's file into that shape.

Standard library only (used by the fake sensor on the CI runner and by the cloud check inside
the image).
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
RECORDING = REPO / "ouster-ros/ouster-sdk/tests/pcaps/OS-2-128-U1_v2.3.0_1024x10"
# Pinned: the smoke test's expectations are computed from exactly these bytes.
RECORDING_SHA256 = {
    ".pcap": "66ef581f62f7cec8d8067efff38f46687672c6b39cdaa65095e795268d3362b5",
    ".json": "7e77f644a80e998a9c60ac67309200c2b299fcaf6a140c8bec209af14372334a",
}

PROFILE = "RNG19_RFL8_SIG16_NIR16"
PACKET_HEADER = 32
PACKET_FOOTER = 32
COLUMN_HEADER = 12
PIXEL_BYTES = 12
LIDAR_PORT = 7502  # the recording's destination ports
IMU_PORT = 7503


@dataclass
class Recording:
    meta: dict  # legacy (flat) metadata as recorded
    lidar: list[bytes]  # UDP payloads, in order
    imu: list[bytes]

    @property
    def width(self) -> int:
        return self.meta["data_format"]["columns_per_frame"]

    @property
    def height(self) -> int:
        return self.meta["data_format"]["pixels_per_column"]

    @property
    def columns_per_packet(self) -> int:
        return self.meta["data_format"]["columns_per_packet"]

    @property
    def shifts(self) -> list[int]:
        return self.meta["data_format"]["pixel_shift_by_row"]

    @property
    def lidar_packet_size(self) -> int:
        return (PACKET_HEADER + PACKET_FOOTER
                + self.columns_per_packet * (COLUMN_HEADER + self.height * PIXEL_BYTES))


def read_pcap(path: Path) -> list[tuple[int, bytes]]:
    """(destination port, UDP payload) for each IPv4/UDP frame of a classic little-endian pcap."""
    data = path.read_bytes()
    magic, _, _, _, _, _, linktype = struct.unpack_from("<IHHiIII", data, 0)
    if magic != 0xA1B2C3D4 or linktype != 1:
        raise ValueError(f"{path}: not a little-endian Ethernet pcap")
    out = []
    off = 24
    while off < len(data):
        _, _, incl, _ = struct.unpack_from("<IIII", data, off)
        frame = data[off + 16:off + 16 + incl]
        off += 16 + incl
        if struct.unpack_from(">H", frame, 12)[0] != 0x0800:
            continue
        ihl = (frame[14] & 0x0F) * 4
        if frame[14 + 9] != 17:
            continue
        udp = 14 + ihl
        _, dport, length = struct.unpack_from(">HHH", frame, udp)
        out.append((dport, frame[udp + 8:udp + length]))
    return out


def load(base: Path = RECORDING) -> Recording:
    meta = json.loads(Path(str(base) + ".json").read_text())
    packets = read_pcap(Path(str(base) + ".pcap"))
    rec = Recording(meta=meta, lidar=[p for d, p in packets if d == LIDAR_PORT],
                    imu=[p for d, p in packets if d == IMU_PORT])
    if meta["data_format"]["udp_profile_lidar"] != PROFILE:
        raise ValueError(f"expected profile {PROFILE}")
    for p in rec.lidar:
        if len(p) != rec.lidar_packet_size:
            raise ValueError(f"lidar packet of {len(p)} bytes, expected {rec.lidar_packet_size}")
    return rec


def nested_metadata(rec: Recording, udp_dest: str = "", lidar_port: int = 7502,
                    imu_port: int = 7503, timestamp_mode: str = "TIME_FROM_INTERNAL_OSC") -> dict:
    """The recording's metadata in the format a sensor's api/v1/sensor/metadata returns."""
    m = rec.meta
    fmt = dict(m["data_format"])
    return {
        "sensor_info": {
            "build_date": m["build_date"], "build_rev": m["build_rev"], "image_rev": m["image_rev"],
            "initialization_id": m["initialization_id"], "prod_line": m["prod_line"],
            "prod_pn": m["prod_pn"], "prod_sn": m["prod_sn"], "status": "RUNNING",
        },
        "beam_intrinsics": {
            "beam_altitude_angles": m["beam_altitude_angles"],
            "beam_azimuth_angles": m["beam_azimuth_angles"],
            "lidar_origin_to_beam_origin_mm": m["lidar_origin_to_beam_origin_mm"],
        },
        "imu_intrinsics": {"imu_to_sensor_transform": m["imu_to_sensor_transform"]},
        "lidar_intrinsics": {"lidar_to_sensor_transform": m["lidar_to_sensor_transform"]},
        "lidar_data_format": fmt,
        "calibration_status": {"reflectivity": {"timestamp": "2022-04-14T21:11:47", "valid": True}},
        "config_params": config_params(rec, udp_dest, lidar_port, imu_port, timestamp_mode),
    }


def config_params(rec: Recording, udp_dest: str, lidar_port: int, imu_port: int,
                  timestamp_mode: str) -> dict:
    fmt = rec.meta["data_format"]
    return {
        "azimuth_window": [0, 360000], "columns_per_packet": fmt["columns_per_packet"],
        "lidar_mode": rec.meta["lidar_mode"], "multipurpose_io_mode": "OFF",
        "nmea_baud_rate": "BAUD_9600", "nmea_ignore_valid_char": 0,
        "nmea_in_polarity": "ACTIVE_HIGH", "nmea_leap_seconds": 0, "operating_mode": "NORMAL",
        "phase_lock_enable": False, "phase_lock_offset": 0, "signal_multiplier": 1,
        "sync_pulse_in_polarity": "ACTIVE_HIGH", "sync_pulse_out_angle": 360,
        "sync_pulse_out_frequency": 1, "sync_pulse_out_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_pulse_width": 10, "timestamp_mode": timestamp_mode, "udp_dest": udp_dest,
        "udp_port_imu": imu_port, "udp_port_lidar": lidar_port,
        "udp_profile_imu": fmt["udp_profile_imu"], "udp_profile_lidar": fmt["udp_profile_lidar"],
    }


# ---------------------------------------------------------------------------------------------
# Packet contents.
# ---------------------------------------------------------------------------------------------


def column_offsets(rec: Recording) -> list[int]:
    step = COLUMN_HEADER + rec.height * PIXEL_BYTES
    return [PACKET_HEADER + i * step for i in range(rec.columns_per_packet)]


def frame_id(packet: bytes) -> int:
    return struct.unpack_from("<H", packet, 2)[0]


def restamp_lidar(rec: Recording, packet: bytes, new_frame_id: int, base_ns: int,
                  first_ts: int) -> bytes:
    """The packet with another frame id and every column timestamp moved to base_ns + (ts -
    first_ts). Pixel data is untouched."""
    out = bytearray(packet)
    struct.pack_into("<H", out, 2, new_frame_id & 0xFFFF)
    for off in column_offsets(rec):
        ts = struct.unpack_from("<Q", packet, off)[0]
        struct.pack_into("<Q", out, off, base_ns + max(0, ts - first_ts))
    return bytes(out)


def restamp_imu(packet: bytes, base_ns: int, first_ts: int) -> bytes:
    out = bytearray(packet)
    for off in (0, 8, 16):
        ts = struct.unpack_from("<Q", packet, off)[0]
        struct.pack_into("<Q", out, off, base_ns + max(0, ts - first_ts))
    return bytes(out)


def first_timestamp(rec: Recording) -> int:
    return min(struct.unpack_from("<Q", p, off)[0] for p in rec.lidar for off in column_offsets(rec))


def staggered_image(rec: Recording) -> dict[str, list[list[int]]]:
    """Per channel, a height x width image in the sensor's column order (measurement id)."""
    h, w = rec.height, rec.width
    chans = {c: [[0] * w for _ in range(h)] for c in ("range", "reflectivity", "signal", "near_ir")}
    for p in rec.lidar:
        for off in column_offsets(rec):
            mid, status = struct.unpack_from("<HH", p, off + 8)
            if not status & 1 or mid >= w:
                continue
            for u in range(h):
                px = off + COLUMN_HEADER + u * PIXEL_BYTES
                rng, refl, _, sig, nir = struct.unpack_from("<IBBHH", p, px)
                chans["range"][u][mid] = rng & 0x7FFFF
                chans["reflectivity"][u][mid] = refl
                chans["signal"][u][mid] = sig
                chans["near_ir"][u][mid] = nir
    return chans


def destagger(image: list[list[int]], shifts: list[int], sign: int) -> list[list[int]]:
    """Row u shifted by sign * shifts[u] columns (circular): out[u][v] = img[u][(v - s) % w]."""
    w = len(image[0])
    return [[row[(v - sign * shifts[u]) % w] for v in range(w)] for u, row in enumerate(image)]

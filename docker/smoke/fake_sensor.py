#!/usr/bin/env python3
"""A fake Ouster sensor for the smoke test: the HTTP API the Ouster SDK uses to configure a sensor
and read its metadata, and UDP lidar and IMU data replayed from a real recording.

HTTP (port 80 on --sensor-ip, as a sensor on firmware 2.3): api/v1/system/firmware,
api/v1/sensor/metadata[/<part>], api/v1/sensor/cmd/get_config_param?args=active|staged,
set_config_param, set_udp_dest_auto (the requester's address becomes udp_dest), reinitialize
(staged becomes active), save_config_params. Every request is logged.

UDP: once the active configuration has a udp_dest, the recorded frame (ouster_packets.py) is sent
to udp_dest at udp_port_lidar (and the IMU packets to udp_port_imu) ten times a second, each time
with the next frame id and the column timestamps moved to the current time (nanoseconds since
the epoch, as a PTP-synchronised sensor would send). Pixel data is sent as recorded.

Writes a JSON summary (requests, active configuration, packets sent) to --summary on exit.
Standard library only. Needs to bind port 80 (run with sudo on the CI runner).
"""

from __future__ import annotations

import argparse
import copy
import json
import signal
import socket
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ouster_packets as op  # noqa: E402

FRAME_PERIOD = 0.1


class Sensor:
    def __init__(self, rec: op.Recording):
        self.rec = rec
        self.lock = threading.Lock()
        self.active = op.config_params(rec, "", op.LIDAR_PORT, op.IMU_PORT,
                                       "TIME_FROM_INTERNAL_OSC")
        self.staged = copy.deepcopy(self.active)
        self.requests: list[str] = []
        self.reinits = 0
        self.persisted = 0
        self.sent = {"lidar": 0, "imu": 0, "frames": 0}

    def metadata(self) -> dict:
        with self.lock:
            cfg = copy.deepcopy(self.active)
        meta = op.nested_metadata(self.rec)
        meta["config_params"] = cfg
        return meta


def make_handler(sensor: Sensor):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # one line per request on stdout instead
            pass

        def reply(self, code: int, body: str, ctype: str = "application/json") -> None:
            data = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):  # noqa: N802 (http.server API)
            path, _, query = self.path.lstrip("/").partition("?")
            with sensor.lock:
                sensor.requests.append(self.path)
            print(f"fake-sensor: GET {self.path[:160]}", flush=True)
            meta_parts = op.nested_metadata(sensor.rec)
            if path == "api/v1/system/firmware":
                return self.reply(200, json.dumps({"fw": sensor.rec.meta["image_rev"]}))
            if path == "api/v1/sensor/metadata":
                return self.reply(200, json.dumps(sensor.metadata()))
            if path.startswith("api/v1/sensor/metadata/"):
                part = path.rsplit("/", 1)[1]
                if part == "config_params":
                    return self.reply(200, json.dumps(sensor.metadata()["config_params"]))
                if part in meta_parts:
                    return self.reply(200, json.dumps(meta_parts[part]))
                return self.reply(404, "{}")
            if path == "api/v1/sensor/cmd/get_config_param":
                args = urllib.parse.unquote(query.partition("=")[2])
                with sensor.lock:
                    if args == "active":
                        return self.reply(200, json.dumps(sensor.active))
                    if args == "staged":
                        return self.reply(200, json.dumps(sensor.staged))
                    if args in sensor.active:
                        return self.reply(200, json.dumps(sensor.active[args]))
                return self.reply(400, '"unknown parameter"')
            if path == "api/v1/sensor/cmd/set_config_param":
                raw = query.partition("=")[2]
                key, _, value = raw.partition("+")
                value = urllib.parse.unquote(value)
                try:
                    parsed = json.loads(value)
                except ValueError:
                    parsed = value
                with sensor.lock:
                    if key == ".":
                        if not isinstance(parsed, dict):
                            return self.reply(400, '"expected an object"')
                        sensor.staged.update(parsed)
                    else:
                        sensor.staged[key] = parsed
                return self.reply(200, '"set_config_param"')
            if path == "api/v1/sensor/cmd/set_udp_dest_auto":
                with sensor.lock:
                    sensor.staged["udp_dest"] = self.client_address[0]
                return self.reply(200, "{}")
            if path == "api/v1/sensor/cmd/reinitialize":
                with sensor.lock:
                    sensor.active = copy.deepcopy(sensor.staged)
                    sensor.reinits += 1
                    shown = {k: sensor.active[k] for k in ("udp_dest", "udp_port_lidar",
                                                           "udp_port_imu", "timestamp_mode",
                                                           "operating_mode", "lidar_mode")}
                print(f"fake-sensor: reinitialized with {json.dumps(shown, sort_keys=True)}",
                      flush=True)
                return self.reply(200, "{}")
            if path == "api/v1/sensor/cmd/save_config_params":
                with sensor.lock:
                    sensor.persisted += 1
                return self.reply(200, "{}")
            return self.reply(404, "{}")

    return Handler


def sender(sensor: Sensor, sensor_ip: str, stop: threading.Event) -> None:
    rec = sensor.rec
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
    sock.bind((sensor_ip, 0))
    first = op.first_timestamp(rec)
    imu_first = min(int.from_bytes(p[0:8], "little") for p in rec.imu) if rec.imu else 0
    fid = op.frame_id(rec.lidar[0])
    imu_every = max(1, len(rec.lidar) // max(1, len(rec.imu)))
    while not stop.is_set():
        start = time.monotonic()
        with sensor.lock:
            dest = sensor.active.get("udp_dest") or ""
            lport = int(sensor.active.get("udp_port_lidar") or 0)
            iport = int(sensor.active.get("udp_port_imu") or 0)
        if dest and lport:
            base = time.time_ns()
            fid = (fid + 1) & 0xFFFF
            imu_i = 0
            for i, packet in enumerate(rec.lidar):
                sock.sendto(op.restamp_lidar(rec, packet, fid, base, first), (dest, lport))
                sensor.sent["lidar"] += 1
                if iport and i % imu_every == 0 and imu_i < len(rec.imu):
                    sock.sendto(op.restamp_imu(rec.imu[imu_i], base, imu_first), (dest, iport))
                    sensor.sent["imu"] += 1
                    imu_i += 1
                time.sleep(0.0012)
            sensor.sent["frames"] += 1
        stop.wait(max(0.0, FRAME_PERIOD - (time.monotonic() - start)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sensor-ip", required=True)
    ap.add_argument("--http-port", type=int, default=80)
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--summary", default="")
    ap.add_argument("--recording", default=str(op.RECORDING))
    args = ap.parse_args()

    sensor = Sensor(op.load(Path(args.recording)))
    server = ThreadingHTTPServer((args.sensor_ip, args.http_port), make_handler(sensor))
    server.daemon_threads = True
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    tx = threading.Thread(target=sender, args=(sensor, args.sensor_ip, stop), daemon=True)
    tx.start()
    print(f"fake-sensor: OS-2-128 recording at {args.sensor_ip}:{args.http_port} for "
          f"{args.seconds:.0f} s", flush=True)
    stop.wait(args.seconds)
    stop.set()
    tx.join(timeout=5)
    server.shutdown()
    summary = {"requests": sensor.requests, "active": sensor.active, "reinits": sensor.reinits,
               "persisted": sensor.persisted, "sent": sensor.sent}
    print(f"fake-sensor: sent {json.dumps(sensor.sent)}", flush=True)
    if args.summary:
        Path(args.summary).write_text(json.dumps(summary, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

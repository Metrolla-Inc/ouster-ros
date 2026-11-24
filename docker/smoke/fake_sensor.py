#!/usr/bin/env python3
"""A fake Ouster sensor for the smoke test: the HTTP API the Ouster SDK uses to configure a sensor
and read its metadata, and UDP lidar and IMU data replayed from a real recording.

HTTP (port 80 on --sensor-ip). --firmware sets the version the sensor reports (default: the
recording's own, 2.3.0); the API follows the firmware, as the Ouster SDK expects it:

- every firmware: api/v1/system/firmware, api/v1/sensor/metadata[/<part>], GET api/v1/sensor/config
  (the active configuration; SDK 0.16 reads it first in set_config);
- firmware before 3.1: api/v1/sensor/cmd/get_config_param?args=active|staged, set_config_param,
  set_udp_dest_auto (the requester's address becomes udp_dest), reinitialize (staged becomes
  active), save_config_params;
- firmware 3.1 and later: GET api/v1/sensor/config?staging=true|false, POST api/v1/sensor/config
  with ?staging=true&reinit=false (merge the JSON body into the staged configuration;
  "udp_dest": "@auto" means the requester's address), ?staging=true&reinit=true (staged becomes
  active) and ?persist=true (save);
- firmware 2.5 and later except 3.0: GET api/v1/user/data (empty).

Every request is logged.

UDP: once the active configuration has a udp_dest, the recorded frame (ouster_packets.py) is sent
to udp_dest at udp_port_lidar (and the IMU packets to udp_port_imu) ten times a second, each time
with the next frame id and the column timestamps moved to the current time (nanoseconds since
the epoch, as a PTP-synchronised sensor would send). Pixel data is sent as recorded.

Writes a JSON summary (requests, the configuration before and after, packets sent) to --summary
on exit.
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


def version_tuple(text: str) -> tuple[int, int, int]:
    parts = text.split(".")
    if len(parts) != 3 or not all(p.isdigit() for p in parts):
        raise ValueError(f"firmware {text!r}: expected X.Y.Z")
    return int(parts[0]), int(parts[1]), int(parts[2])


class Sensor:
    def __init__(self, rec: op.Recording, firmware: str | None = None):
        self.rec = rec
        self.image_rev = op.with_firmware(rec.meta["image_rev"], firmware) if firmware \
            else rec.meta["image_rev"]
        self.version = version_tuple(self.image_rev.partition("-v")[2].split("+")[0])
        self.staging_api = self.version >= (3, 1, 0)  # SDK: MINIMUM_STAGING_VERSION
        self.user_data_api = self.version >= (2, 5, 0) and self.version[:2] != (3, 0)
        self.lock = threading.Lock()
        self.active = op.config_params(rec, "", op.LIDAR_PORT, op.IMU_PORT,
                                       "TIME_FROM_INTERNAL_OSC")
        self.initial = copy.deepcopy(self.active)
        self.staged = copy.deepcopy(self.active)
        self.requests: list[str] = []
        self.reinits = 0
        self.persisted = 0
        self.sent = {"lidar": 0, "imu": 0, "frames": 0}

    def parts(self) -> dict:
        meta = op.nested_metadata(self.rec)
        meta["sensor_info"]["image_rev"] = self.image_rev
        return meta

    def metadata(self) -> dict:
        with self.lock:
            cfg = copy.deepcopy(self.active)
        meta = self.parts()
        meta["config_params"] = cfg
        return meta

    def reinitialize(self) -> None:
        with self.lock:
            self.active = copy.deepcopy(self.staged)
            self.reinits += 1
            shown = {k: self.active.get(k) for k in ("udp_dest", "udp_port_lidar", "udp_port_imu",
                                                     "timestamp_mode", "operating_mode",
                                                     "lidar_mode", "multipurpose_io_mode")}
        print(f"fake-sensor: reinitialized with {json.dumps(shown, sort_keys=True)}", flush=True)


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

        def note(self, method: str, body: bytes = b"") -> None:
            line = f"{method} {self.path}" + (f" {body.decode(errors='replace')}" if body else "")
            with sensor.lock:
                sensor.requests.append(line)
            print(f"fake-sensor: {line[:200]}", flush=True)

        def do_GET(self):  # noqa: N802 (http.server API)
            self.note("GET")
            path, _, query = self.path.lstrip("/").partition("?")
            params = urllib.parse.parse_qs(query)
            meta_parts = sensor.parts()
            if path == "api/v1/system/firmware":
                return self.reply(200, json.dumps({"fw": sensor.image_rev}))
            if path == "api/v1/sensor/metadata":
                return self.reply(200, json.dumps(sensor.metadata()))
            if path.startswith("api/v1/sensor/metadata/"):
                part = path.rsplit("/", 1)[1]
                if part == "config_params":
                    return self.reply(200, json.dumps(sensor.metadata()["config_params"]))
                if part in meta_parts:
                    return self.reply(200, json.dumps(meta_parts[part]))
                return self.reply(404, "{}")
            if path == "api/v1/user/data" and sensor.user_data_api:
                return self.reply(200, '""')
            if path == "api/v1/sensor/config":
                staging = params.get("staging", ["false"])[0].lower() == "true"
                if staging and not sensor.staging_api:
                    return self.reply(404, "{}")
                with sensor.lock:
                    return self.reply(200, json.dumps(sensor.staged if staging else sensor.active))
            if sensor.staging_api:
                return self.reply(404, "{}")  # the cmd API is gone on these firmware versions
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
                sensor.reinitialize()
                return self.reply(200, "{}")
            if path == "api/v1/sensor/cmd/save_config_params":
                with sensor.lock:
                    sensor.persisted += 1
                return self.reply(200, "{}")
            return self.reply(404, "{}")

        def do_POST(self):  # noqa: N802 (http.server API)
            length = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(length) if length else b""
            self.note("POST", body)
            path, _, query = self.path.lstrip("/").partition("?")
            params = {k.lower(): v[0].lower() for k, v in urllib.parse.parse_qs(query).items()}
            if path != "api/v1/sensor/config" or not sensor.staging_api:
                return self.reply(404, "{}")
            try:
                update = json.loads(body or b"{}")
            except ValueError:
                return self.reply(400, '"invalid json"')
            if not isinstance(update, dict):
                return self.reply(400, '"expected an object"')
            if update.get("udp_dest") == "@auto":
                update["udp_dest"] = self.client_address[0]
            with sensor.lock:
                sensor.staged.update(update)
            if params.get("reinit") == "true":
                sensor.reinitialize()
            if params.get("persist") == "true":
                with sensor.lock:
                    sensor.persisted += 1
            return self.reply(200, "{}")

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
    ap.add_argument("--firmware", default="",
                    help="firmware version to report, X.Y.Z (default: the recording's, 2.3.0)")
    args = ap.parse_args()

    sensor = Sensor(op.load(Path(args.recording)), args.firmware or None)
    server = ThreadingHTTPServer((args.sensor_ip, args.http_port), make_handler(sensor))
    server.daemon_threads = True
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    tx = threading.Thread(target=sender, args=(sensor, args.sensor_ip, stop), daemon=True)
    tx.start()
    print(f"fake-sensor: OS-2-128 recording at {args.sensor_ip}:{args.http_port}, firmware "
          f"{sensor.image_rev}, for {args.seconds:.0f} s", flush=True)
    stop.wait(args.seconds)
    stop.set()
    tx.join(timeout=5)
    server.shutdown()
    summary = {"firmware": sensor.image_rev, "requests": sensor.requests,
               "initial": sensor.initial, "active": sensor.active, "reinits": sensor.reinits,
               "persisted": sensor.persisted, "sent": sensor.sent}
    print(f"fake-sensor: sent {json.dumps(sensor.sent)}", flush=True)
    if args.summary:
        Path(args.summary).write_text(json.dumps(summary, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

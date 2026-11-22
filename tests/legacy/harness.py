"""Run an entrypoint script of this image outside the image, in a sandbox, and record what it
does: exit code, stdout, stderr, the command it execs (argv) and the environment that command
gets, including which setup files were sourced and in what order.

The sandbox is a temporary directory laid out like the image: /opt/ros/jazzy/setup.bash and
/opt/ros2_ws/install/setup.bash (each only records that it was sourced), and
/opt/metrolla/ouster/contract-entrypoint.sh as a recorder. The image's ENV (ROS_DISTRO=jazzy from
the ros:jazzy base image, ROS2_WS=/opt/ros2_ws from Dockerfile.jazzy) is given to the script, the
script's absolute paths are rewritten into the sandbox, the exec'd command is a recorder on PATH,
and every sandbox path is mapped back in the record, so a record reads as if taken in the image.

Used by tests/legacy (goldens of the fleet environments).
"""

from __future__ import annotations

import json
import subprocess
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
REWRITES = ("/opt/ros/", "/opt/metrolla/")
IMAGE_ENV = {"ROS_DISTRO": "jazzy", "ROS2_WS": "/opt/ros2_ws"}
BASE_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C"}
RECORDED_ENV = ("SOURCED", "ROS_DISTRO", "ROS2_WS", "METROLLA_DRIVER_CONTRACT")
DEFAULT_ARGV = ["ros2", "launch", "ouster_ros", "driver.launch.py",
                "sensor_hostname:=192.168.1.201", "viz:=false"]  # Dockerfile.jazzy CMD

RECORDER = """#!/usr/bin/env python3
import json, os, sys
with open(os.environ["HARNESS_RECORD"], "w") as f:
    json.dump({"argv": [os.path.basename(sys.argv[0])] + sys.argv[1:], "env": dict(os.environ)}, f)
"""


def _layout(root: Path) -> None:
    for setup, label in (("opt/ros/jazzy/setup.bash", "/opt/ros/jazzy"),
                         ("opt/ros2_ws/install/setup.bash", "/opt/ros2_ws/install")):
        path = root / setup
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f'export SOURCED="${{SOURCED:+$SOURCED:}}{label}"\n')
    bindir = root / "bin"
    bindir.mkdir()
    (bindir / "ros2").write_text(RECORDER)
    (bindir / "ros2").chmod(0o755)
    contract = root / "opt/metrolla/ouster/contract-entrypoint.sh"
    contract.parent.mkdir(parents=True)
    contract.write_text(RECORDER)
    contract.chmod(0o755)


def run(script: Path, env: dict[str, str], argv: list[str]) -> dict:
    with tempfile.TemporaryDirectory(prefix="ouster-legacy-") as tmp:
        root = Path(tmp)
        _layout(root)
        text = script.read_text()
        for prefix in REWRITES:
            text = text.replace(prefix, str(root) + prefix)
        sandboxed = root / "entrypoint.sh"
        sandboxed.write_text(text)
        record_file = root / "record.json"
        run_env = dict(BASE_ENV)
        run_env["PATH"] = f"{root}/bin:" + run_env["PATH"]
        run_env["HARNESS_RECORD"] = str(record_file)
        run_env.update(IMAGE_ENV)
        run_env["ROS2_WS"] = str(root) + IMAGE_ENV["ROS2_WS"]
        run_env.update(env)
        proc = subprocess.run(["bash", str(sandboxed), *argv], env=run_env, capture_output=True,
                              text=True, timeout=60, check=False)

        def back(value: str) -> str:
            return value.replace(str(root), "")

        record = None
        if record_file.exists():
            raw = json.loads(record_file.read_text())
            exec_env = {k: back(v) for k, v in raw["env"].items()
                        if k in RECORDED_ENV or k in env}
            record = {"argv": [back(a) for a in raw["argv"]], "env": dict(sorted(exec_env.items()))}
        return {"exit": proc.returncode, "stdout": back(proc.stdout), "stderr": back(proc.stderr),
                "exec": record}


def read_case(path: Path) -> tuple[dict[str, str], list[str]]:
    """KEY=VALUE lines (# comments); a line 'argv: a b c' sets the command (default: the CMD)."""
    env: dict[str, str] = {}
    argv = list(DEFAULT_ARGV)
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("argv:"):
            argv = line[len("argv:"):].split()
            continue
        key, sep, value = line.partition("=")
        assert sep, f"{path}: not KEY=VALUE: {line!r}"
        env[key] = value
    return env, argv


def dump(record: dict) -> str:
    return json.dumps(record, indent=2, sort_keys=True) + "\n"

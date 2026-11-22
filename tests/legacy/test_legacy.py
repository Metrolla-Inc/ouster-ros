"""Legacy mode stays exactly the v1.0.0 image's behaviour (the image two fleets pin).

Goldens: for each environment in cases/*.env (taken from the fleet compose files that pin
ghcr.io/metrolla-inc/ouster-ros-driver:v1.0.0, and from this repository's docker-compose.yml), the
record of running the entrypoint (exit code, stdout, stderr, the exec'd command and its
environment, which setup files were sourced) as produced by the **v1.0.0 entrypoint**
(entrypoint-v1.0.0.sh, byte-identical to git tag v1.0.0). The current entrypoint.sh must produce
the same record.

REGEN=1 rewrites the goldens from the v1.0.0 script, never from the current one.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from harness import REPO, dump, read_case, run

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))
REFERENCE = HERE / "entrypoint-v1.0.0.sh"
CURRENT = REPO / "entrypoint.sh"
V100_BLOB = "ac03bd8c588d1751fcdbe7c84d0feab8abd261f4"  # git rev-parse v1.0.0:entrypoint.sh
SHEBANG = "#!/bin/bash\n"
BLOCK_END = "# --- end of the Metrolla contract block; the v1.0.0 entrypoint follows ---\n"


def git_blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def test_reference_is_the_v1_0_0_entrypoint():
    assert git_blob_id(REFERENCE.read_bytes()) == V100_BLOB


def test_entrypoint_is_v1_0_0_plus_the_opt_in_block_only():
    current = CURRENT.read_text()
    reference = REFERENCE.read_text()
    assert current.startswith(SHEBANG) and reference.startswith(SHEBANG)
    head, sep, tail = current.partition(BLOCK_END)
    assert sep, "entrypoint.sh has no contract block end marker"
    assert tail == reference[len(SHEBANG):], "the legacy part of entrypoint.sh changed"
    block = [line for line in head[len(SHEBANG):].splitlines() if not line.startswith("#")]
    assert block == [
        'if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then',
        '    exec /opt/metrolla/ouster/contract-entrypoint.sh "$@"',
        "fi",
    ]


def test_there_are_fleet_cases():
    assert len(CASES) >= 6


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_legacy_golden(case: Path):
    env, argv = read_case(case)
    golden = HERE / "golden" / (case.stem + ".json")
    reference = dump(run(REFERENCE, env, argv))
    if os.environ.get("REGEN"):
        golden.write_text(reference)
    expected = golden.read_text()
    assert reference == expected, "the v1.0.0 entrypoint no longer gives its golden (harness changed?)"
    assert dump(run(CURRENT, env, argv)) == expected, "legacy behaviour changed"


def test_fleet_default_runs_the_image_command():
    """The fleets set no command: the image's CMD runs, with ROS and the workspace sourced."""
    env, argv = read_case(HERE / "cases" / "fleet-compose-default-cmd.env")
    record = run(CURRENT, env, argv)
    assert record["exit"] == 0
    assert record["exec"]["argv"] == ["ros2", "launch", "ouster_ros", "driver.launch.py",
                                      "sensor_hostname:=192.168.1.201", "viz:=false"]
    assert record["exec"]["env"]["SOURCED"] == "/opt/ros/jazzy:/opt/ros2_ws/install"


@pytest.mark.parametrize("value", ["1", "2", "yes", "true", " "])
def test_any_other_value_hands_over_to_the_contract(value):
    """Set and not "0": the contract entrypoint gets the command (and refuses values it does not
    know, tests/regression); nothing of the legacy entrypoint runs."""
    env, argv = read_case(HERE / "cases" / "fleet-compose-default-cmd.env")
    env["METROLLA_DRIVER_CONTRACT"] = value
    record = run(CURRENT, env, argv)
    assert record["exec"]["argv"] == ["contract-entrypoint.sh"] + argv
    assert "SOURCED" not in record["exec"]["env"]

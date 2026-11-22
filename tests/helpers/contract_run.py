"""Run ouster_env.py's command line in-process against this repository's vendor parameter file
(ouster-ros/config/driver_params.yaml, the file the image installs) and capture what it prints,
in the format of tests/regression/golden/*.golden, which the smoke test also produces in the
image."""

from __future__ import annotations

import contextlib
import io
from pathlib import Path

import ouster_env

REPO = Path(__file__).resolve().parents[2]
VENDOR_PARAMS = REPO / "ouster-ros/config/driver_params.yaml"


def read_env(path: Path) -> dict[str, str]:
    env: dict[str, str] = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        key, sep, value = line.partition("=")
        assert sep, f"{path}: not KEY=VALUE: {line!r}"
        env[key] = value
    return env


def run_main(argv: list[str], env: dict[str, str], monkeypatch) -> tuple[int, str, str]:
    monkeypatch.setattr(ouster_env, "VENDOR_PARAMS", str(VENDOR_PARAMS))
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = ouster_env.main(argv, env)
    return code, out.getvalue(), err.getvalue()


def golden_text(code: int, out: str, err: str) -> str:
    return f"exit {code}\n--- stdout\n{out}--- stderr\n{err}"

"""Goldens of the contract: for each environment in cases/*.env, exactly what `check` prints
(exit code, rendered parameter file and exec line, or one refusal line per problem). The smoke
test runs the same cases in the built image and compares with the same files.

REGEN=1 rewrites the goldens (review the diff: a changed golden is a changed contract).
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from contract_run import golden_text, read_env, run_main

HERE = Path(__file__).resolve().parent
CASES = sorted((HERE / "cases").glob("*.env"))


def test_there_are_cases():
    assert len(CASES) >= 15


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_contract_golden(case: Path, monkeypatch):
    code, out, err = run_main(["check"], read_env(case), monkeypatch)
    text = golden_text(code, out, err)
    golden = HERE / "golden" / (case.stem + ".golden")
    if os.environ.get("REGEN"):
        golden.write_text(text)
    assert text == golden.read_text()
    assert code == (1 if case.stem.startswith("refused-") else 0)

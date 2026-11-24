"""The image workflow's tag step (.github/workflows/build-and-push.yml), run under bash as the push
job runs it: Metrolla tags are v<vendor version>-metrolla.<n> (owner, 22 Nov 2026) and never
reuse a vendor's tag; anything else is refused before the registry is touched."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
WORKFLOW = REPO / ".github/workflows/build-and-push.yml"
SHA = "0123456789abcdef0123456789abcdef01234567"


def tag_step_script() -> str:
    doc = yaml.safe_load(WORKFLOW.read_text())
    job = doc["jobs"]["build-and-push"]
    return next(s for s in job["steps"] if s.get("id") == "tag")["run"]


def run_tag_step(tmp_path: Path, ref_name: str, ref_type: str = "tag",
                 head: str = SHA) -> tuple[int, str, str]:
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    (fake / "git").write_text(f"#!/bin/sh\necho {head}\n")  # git rev-parse HEAD
    (fake / "git").chmod(0o755)
    out = tmp_path / "out"
    out.write_text("")
    env = dict(os.environ, PATH=f"{fake}:{os.environ['PATH']}", REF_TYPE=ref_type,
               REF_NAME=ref_name, GITHUB_SHA=SHA, GITHUB_OUTPUT=str(out))
    proc = subprocess.run(["bash", "-e", "-c", tag_step_script()], env=env, capture_output=True,
                          text=True)
    return proc.returncode, out.read_text(), proc.stderr


@pytest.mark.parametrize("good", ["v0.15.1-metrolla.1", "v0.15.1-metrolla.12",
                                  "vjazzy-0.15.1-metrolla.1", "v1.0.0-metrolla.3"])
def test_metrolla_tags_are_accepted_and_name_the_image(tmp_path, good):
    code, out, err = run_tag_step(tmp_path, good)
    assert code == 0, err
    assert out == f"tag={good}\n"


@pytest.mark.parametrize("bad", [
    "v1.0.0",  # the released fleet tag's pattern: a plain version is refused from now on
    "v0.15.1",  # a vendor version without the Metrolla suffix
    "vjazzy-0.15.1", "v0.15.1-metrolla", "v0.15.1-metrolla.0", "v0.15.1-metrolla.01",
    "v0.15.1-metrolla.1-rc", "v0.15.1-Metrolla.1", "0.15.1-metrolla.1", "v-metrolla.1",
    "jazzy-0.15.1", "vros2-v0.14.0",
])
def test_other_tags_are_refused(tmp_path, bad):
    code, out, err = run_tag_step(tmp_path, bad)
    assert code == 1 and out == "", bad
    assert "v<vendor version>-metrolla.<n>" in err


def test_a_branch_is_refused(tmp_path):
    code, out, err = run_tag_step(tmp_path, "ros2", ref_type="branch")
    assert code == 1 and out == "" and "tags only" in err


def test_a_checkout_that_is_not_the_tagged_commit_is_refused(tmp_path):
    code, out, err = run_tag_step(tmp_path, "v0.15.1-metrolla.1", head="f" * 40)
    assert code == 1 and out == "" and "not the tagged commit" in err


def test_the_overwrite_refusal_stays_before_the_push():
    doc = yaml.safe_load(WORKFLOW.read_text())
    names = [s.get("name", "") for s in doc["jobs"]["build-and-push"]["steps"]]
    tag = next(i for i, s in enumerate(doc["jobs"]["build-and-push"]["steps"]) if s.get("id") == "tag")
    refuse = names.index("Refuse to overwrite an existing image tag")
    push = next(i for i, n in enumerate(names) if n.startswith("Build and push"))
    assert tag < refuse < push

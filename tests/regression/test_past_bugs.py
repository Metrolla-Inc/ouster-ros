"""Past bugs: each test fails on the code before its fix (git show v1.0.0:<file>)."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
WORKFLOW = REPO / ".github/workflows/build-and-push.yml"


def load_workflow(text: str) -> dict:
    data = yaml.safe_load(text)
    data["on"] = data.pop(True, data.get("on"))  # YAML 1.1 reads the key `on` as True
    return data


def steps(job: dict) -> list[dict]:
    return job.get("steps", [])


def check_tag_build_uses_the_tagged_commit(data: dict) -> list[str]:
    """Bug (v1.0.0 workflow): every push, tags included, checked out `ref: ros2`, so the image
    tagged vX was built from whatever the ros2 branch held when the job ran, not from tag vX."""
    problems = []
    for name, job in data["jobs"].items():
        for step in steps(job):
            if "actions/checkout" in step.get("uses", "") and "ref" in (step.get("with") or {}):
                problems.append(f"{name}: checkout pins ref {step['with']['ref']!r}")
    return problems


def check_push_only_for_tags(data: dict) -> list[str]:
    """Bug (v1.0.0 workflow): pushes to ros2, main, dev and to any branch named v* pushed an
    image (ouster-ros-driver:<branch>), and an existing tag was overwritten without a check."""
    problems = []
    push = data["on"].get("push") or {}
    if set(push) - {"tags"} or push.get("tags") != ["v*"]:
        problems.append(f"push trigger is {push!r}, expected tags v* only")
    for name, job in data["jobs"].items():
        pushes = any((s.get("with") or {}).get("push") is True for s in steps(job))
        logs_in = any("docker/login-action" in s.get("uses", "") for s in steps(job))
        if pushes or logs_in:
            cond = job.get("if", "")
            if "refs/tags/v" not in cond or "github.event_name == 'push'" not in cond:
                problems.append(f"{name}: pushes or logs in without the tag-push condition")
            if not any("Refuse to overwrite" in s.get("name", "") for s in steps(job)):
                problems.append(f"{name}: no refusal to overwrite an existing tag")
    return problems


def check_actions_pinned(data: dict) -> list[str]:
    return [f"{name}: {s['uses']}" for name, job in data["jobs"].items() for s in steps(job)
            if "uses" in s and not re.search(r"@[0-9a-f]{40}$", s["uses"])]


def old(path: str) -> str | None:
    try:
        return subprocess.run(["git", "show", f"v1.0.0:{path}"], cwd=REPO, capture_output=True,
                              text=True, check=True).stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None


@pytest.mark.parametrize("check", [check_tag_build_uses_the_tagged_commit, check_push_only_for_tags,
                                   check_actions_pinned])
def test_image_workflow(check):
    assert check(load_workflow(WORKFLOW.read_text())) == []


@pytest.mark.parametrize("check", [check_tag_build_uses_the_tagged_commit, check_push_only_for_tags,
                                   check_actions_pinned])
def test_the_checks_fail_on_the_v1_0_0_workflow(check):
    text = old(".github/workflows/build-and-push.yml")
    if text is None:
        pytest.skip("tag v1.0.0 not fetched")
    assert check(load_workflow(text)) != []


def check_sdk_never_fetched_from_a_branch(dockerfile: str) -> list[str]:
    """Bug (v1.0.0 Dockerfile.jazzy): without the submodule the build cloned the SDK's moving
    default branch (ouster_example) and built that, an SDK version nobody tested with this
    driver. Now the build stops."""
    problems = []
    if re.search(r"git clone", dockerfile):
        problems.append("Dockerfile clones a repository during the build")
    if "ouster-sdk/CMakeLists.txt" not in dockerfile or "exit 1" not in dockerfile:
        problems.append("no check that the SDK submodule is present")
    return problems


def test_dockerfile_refuses_to_build_without_the_submodule():
    assert check_sdk_never_fetched_from_a_branch((REPO / "Dockerfile.jazzy").read_text()) == []


def test_the_dockerfile_check_fails_on_v1_0_0():
    text = old("Dockerfile.jazzy")
    if text is None:
        pytest.skip("tag v1.0.0 not fetched")
    assert check_sdk_never_fetched_from_a_branch(text) != []


def test_ci_workflows_pin_actions():
    for path in (REPO / ".github/workflows").glob("*.yml"):
        assert check_actions_pinned(load_workflow(path.read_text())) == [], path.name


def check_driver_stops_thread_before_its_members(source: str) -> list[str]:
    """Bug (vendor code at v1.0.0, still in the vendor's ros2 head): ~OusterDriver destroyed its
    packet handlers and publishers while the sensor connection thread (stopped only in
    ~OusterSensor) kept calling them, so SIGTERM while packets arrive crashed the node (exit 139,
    seen in the smoke test). ~OusterDriver must stop the thread first."""
    m = re.search(r"~OusterDriver\(\)\s*override\s*\{(.*?)\n    \}", source, re.S)
    if not m:
        return ["no ~OusterDriver() found"]
    return [] if "stop_sensor_connection_thread();" in m.group(1) else [
        "~OusterDriver does not stop the sensor connection thread"]


def test_driver_destructor_stops_the_connection_thread():
    path = REPO / "ouster-ros/src/os_driver_node.cpp"
    assert check_driver_stops_thread_before_its_members(path.read_text()) == []


def test_the_destructor_check_fails_on_v1_0_0():
    text = old("ouster-ros/src/os_driver_node.cpp")
    if text is None:
        pytest.skip("tag v1.0.0 not fetched")
    assert check_driver_stops_thread_before_its_members(text) != []

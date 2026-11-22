# Tests (Metrolla)

Three tiers, separately named jobs in `.github/workflows/ci.yml`. The vendor's own C++ tests
(`ouster-ros/test`) are not run by the Metrolla image build (as in v1.0.0).

| Tier | Command | What it proves |
|---|---|---|
| unit | `python -m pytest tests/unit tests/legacy` (+ shellcheck, hadolint in CI) | the contract (docker/ouster_env.py): every variable, default, refusal, the rendered parameter file against the vendor's `driver_params.yaml`, the exec line, run/check commands; the fake sensor answers the Ouster SDK's configuration sequence and replays the recording unchanged except frame id and timestamps; the cloud check catches a wrong pixel, distance, ring or missing destagger |
| regression | `python -m pytest tests/regression tests/legacy` | goldens: legacy entrypoint records for the fleet environments, generated from the v1.0.0 entrypoint (`tests/legacy/entrypoint-v1.0.0.sh`, git blob pinned) and compared with the current one; contract `check` output for 22 environments (accepted and refused); past bugs (tag builds used the `ros2` branch head; branch pushes pushed images; the Dockerfile cloned the SDK's moving branch when the submodule was missing) |
| smoke | `docker/smoke/smoke.sh IMAGE [OLD_IMAGE]` (needs Docker and sudo) | the real image: entrypoint/command/user unchanged; legacy fleet environments run the v1.0.0 entrypoint; the image's own command publishes `/ouster/points` (frame `os_lidar`) from a fake sensor replaying a real OS-2-128 recording, every pixel checked; contract `check` and refusals byte-identical to the goldens; contract runs as uid 97000, read-only root, tmpfs /tmp, cap-drop ALL, host network: PID 1 is the node, the rendered file is what it reads, clouds on the contract topic and frame with host-time stamps (and ptp stamps 37 s behind UTC), every pixel, the sensor configured with the contract's ports/destination/time mode and nothing persisted, `docker stop` ends it with exit 0 |

Run the Python tiers locally: `python -m venv .venv && .venv/bin/pip install pytest==9.0.1
pyyaml==6.0.3`, then the commands above (the SDK submodule must be checked out: the recording is
`ouster-ros/ouster-sdk/tests/pcaps/OS-2-128-U1_v2.3.0_1024x10.{pcap,json}`, pinned by sha256).
`REGEN=1` rewrites goldens (legacy ones only from the v1.0.0 script).

Not covered here (needs hardware): see docs/METROLLA_DRIVER_CONTRACT.md, "Not proven".

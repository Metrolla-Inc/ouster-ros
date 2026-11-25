# Tests (Metrolla)

Three tiers, separately named jobs in `.github/workflows/ci.yml`. The vendor's own C++ tests
(`ouster-ros/test`) are not run by the Metrolla image build (as in v1.0.0).

| Tier | Command | What it proves |
|---|---|---|
| unit | `python -m pytest tests/unit tests/legacy` (+ shellcheck, hadolint in CI) | the contract (docker/ouster_env.py): every variable, default, refusal, the rendered parameter file against the vendor's `driver_params.yaml` (time-sync wiring left to the sensor), the exec line, run/check commands; the fake sensor answers the Ouster SDK 0.16 configuration sequences (firmware before 3.1: `sensor/cmd` API; 3.1+: staged `sensor/config` API; user data) and replays the recording unchanged except frame id and timestamps; the cloud check catches a wrong pixel, distance, ring, point time `t`, IMU value or missing destagger, accepts the driver's start-up scan (whole columns missing, the rest exact) only as the first cloud and requires every later cloud complete; the image workflow's tag step, run under bash, accepts only `v<vendor version>-metrolla.<n>` tags |
| regression | `python -m pytest tests/regression tests/legacy` | goldens: legacy entrypoint records for the fleet environments, generated from the v1.0.0 entrypoint (`tests/legacy/entrypoint-v1.0.0.sh`, git blob pinned) and compared with the current one; contract `check` output for 22 environments (accepted and refused); past bugs (tag builds used the `ros2` branch head; branch pushes pushed images; the Dockerfile cloned the SDK's moving branch when the SDK was missing; the node crashed on SIGTERM under load; vendor 0.15.1 aborted on the first scan from firmware before 3.2; the smoke cloud check failed on the driver's partial start-up scan, `tests/regression/test_smoke_check.py`) |
| smoke | `docker/smoke/smoke.sh IMAGE [OLD_IMAGE]` (needs Docker and sudo) | the real image: entrypoint/command/user unchanged; legacy fleet environments run the v1.0.0 entrypoint; the image's own command publishes `/ouster/points` (frame `os_lidar`) from a fake sensor replaying a real OS-2-128 recording, every pixel and every point's time of every cloud checked (the first cloud may be the driver's start-up scan: whole columns missing, the rest exact), `/ouster/imu` values checked, and the settings it writes to the sensor listed (docs/VENDOR_DIFF.md); contract `check` and refusals byte-identical to the goldens; contract runs as uid 97000, read-only root, tmpfs /tmp, cap-drop ALL, host network: PID 1 is the node, the rendered file is what it reads, clouds on the contract topic and frame with host-time stamps (and ptp stamps 37 s behind UTC), every pixel and point time, IMU, the sensor configured with the contract's ports/destination/time mode and nothing else changed or persisted (fake firmware 3.1.0 and 2.5.3), `docker stop` ends it with exit 0 (five times under load); a sensor on firmware 2.3.0 is refused by the SDK and left untouched |

Run the Python tiers locally: `python -m venv .venv && .venv/bin/pip install pytest==9.0.1
pyyaml==6.0.3`, then the commands above. The recording is the Ouster SDK's own test data,
`docker/smoke/recording/OS-2-128-U1_v2.3.0_1024x10.{pcap,json}` (pinned by sha256; see the README
there). `REGEN=1` rewrites goldens (legacy ones only from the v1.0.0 script). The regression
tier's past-bug tests compare with tag `v1.0.0` (`git fetch origin
+refs/tags/v1.0.0:refs/tags/v1.0.0`); without it they skip.

Release tags: `v<vendor version>-metrolla.<n>`, for example `v0.15.1-metrolla.1` (vendor release
0.15.1, `jazzy-0.15.1`). The image workflow pushes `ghcr.io/metrolla-inc/ouster-ros-driver:<tag>`
only for such a tag, refuses any other `v*` tag and never overwrites an existing image tag
(`tests/unit/test_workflows.py`). Tags are the owner's to push.

Not covered here (needs hardware): see docs/METROLLA_DRIVER_CONTRACT.md, "Not proven".

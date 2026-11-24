#!/usr/bin/env bash
# Smoke tier of the Ouster driver image (TESTING.md). Needs Docker, python3 and sudo (loopback
# addresses and the fake sensor's HTTP port 80). Usage: docker/smoke/smoke.sh IMAGE [OLD_IMAGE]
#
#   1. Image facts: entrypoint, command and user are what the fleets run today.
#   2. Legacy mode: the fleet environments of tests/legacy/cases never reach the contract; the
#      image's own command (ros2 launch ... driver.launch.py sensor_hostname:=192.168.1.201)
#      publishes /ouster/points from the fake sensor's recorded packets, pixel for pixel and with
#      each point's time t; what the vendor's defaults write to the sensor is listed and checked.
#      With OLD_IMAGE (the released ouster-ros-driver:v1.0.0) the same run is compared with it.
#   3. Contract checks and refusals: stdout/stderr byte-identical to tests/regression/golden.
#   4. Contract runs as the product does it (uid 97000, read-only root, tmpfs /tmp, cap-drop ALL,
#      no-new-privileges, host network and IPC) against the fake sensor: PointCloud2 on the
#      contract's topic and frame, stamps, organized size, every pixel, the sensor configured
#      with the contract's ports, time mode and destination and nothing else changed on it; clean
#      stop on SIGTERM (exit 0). The fake sensor reports firmware 2.5.3 (the SDK's pre-3.1 HTTP
#      API), 3.1.0 (the staged-configuration API) or 3.2.0 (scans with the WINDOW field), and
#      2.3.0 for the firmware floor (5).
set -euo pipefail

IMAGE=${1:?usage: smoke.sh IMAGE [OLD_IMAGE]}
OLD_IMAGE=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SENSOR_IP=192.168.1.201
HOST_ALIAS=192.168.1.100
WORK=$(mktemp -d)
NAME=ouster-smoke-$$
FAKE=
FAILED=0
RECORDING=/repo/docker/smoke/recording/OS-2-128-U1_v2.3.0_1024x10
# Firmware the fake sensor reports to this image's SDK (0.16 refuses firmware before 2.4.0).
FW_OLD_API=2.5.3
FW_NEW_API=3.1.0
FW_WINDOW=3.2.0  # the first firmware whose scans carry the WINDOW field
LEGACY_CMD='["ros2","launch","ouster_ros","driver.launch.py","sensor_hostname:=192.168.1.201","viz:=false"]'

fail() { echo "SMOKE FAIL: $*" >&2; FAILED=1; }
ok() { echo "ok: $*"; }
# expect MESSAGE COMMAND...: ok when the command succeeds, a failure otherwise.
expect() { local msg=$1; shift; if "$@"; then ok "$msg"; else fail "$msg"; fi; }
stop_fake() {
    if [[ -n "$FAKE" ]]; then
        sudo kill -TERM "$FAKE" 2>/dev/null || true
        wait "$FAKE" 2>/dev/null || true
        FAKE=
    fi
}
cleanup() {
    stop_fake
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    sudo ip addr del "$SENSOR_IP/32" dev lo >/dev/null 2>&1 || true
    sudo ip addr del "$HOST_ALIAS/32" dev lo >/dev/null 2>&1 || true
    rm -rf "$WORK"
}
trap cleanup EXIT

# Read KEY=VALUE lines of a case file into docker -e arguments (skips comments and "argv:").
env_args() {
    local line
    ENV_ARGS=()
    while IFS= read -r line; do
        [[ -z "$line" || "$line" == \#* || "$line" == argv:* ]] && continue
        ENV_ARGS+=(-e "$line")
    done < "$1"
}

start_fake() {  # LABEL [FIRMWARE]: empty firmware = the recording's own (2.3.0)
    # shellcheck disable=SC2024  # the log is the runner's file, written by the shell
    sudo python3 "$HERE/fake_sensor.py" --sensor-ip "$SENSOR_IP" --seconds 120 \
        --firmware "${2:-}" --summary "$WORK/$1.fake.json" > "$WORK/$1.fake.log" 2>&1 &
    FAKE=$!
    for _ in $(seq 50); do
        curl -sf "http://$SENSOR_IP/api/v1/system/firmware" > /dev/null && return 0
        sleep 0.2
    done
    fail "fake sensor did not start"; cat "$WORK/$1.fake.log"
}

# check_cloud LABEL TOPIC FRAME [extra check_cloud.py args...]: run the cloud check in the image.
check_cloud() {
    local label=$1 topic=$2 frame=$3; shift 3
    set +e
    # CHECK_USER: the checker runs as the driver's uid. Fast DDS's shared-memory transport (host
    # IPC) only delivers between processes that can open each other's segments.
    docker run --rm --network host --ipc host -e ROS_LOCALHOST_ONLY=1 ${CHECK_USER:+--user "$CHECK_USER"} \
        --tmpfs /tmp -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log \
        -v "$REPO:/repo:ro" --entrypoint bash "$IMAGE" -c \
        'source /opt/ros/jazzy/setup.bash && python3 /repo/docker/smoke/check_cloud.py "$@"' \
        check "$topic" "$frame" --recording "$RECORDING" "$@" > "$WORK/$label.check.json"
    local code=$?
    set -e
    cat "$WORK/$label.check.json"
    return $code
}

diagnose() {  # NODE: lifecycle state and topics, as the driver's uid (after a failed check)
    docker run --rm --network host --ipc host -e ROS_LOCALHOST_ONLY=1 --user 97000:97000 \
        --tmpfs /tmp -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros/log --entrypoint bash "$IMAGE" -c \
        'source /opt/ros/jazzy/setup.bash && echo "lifecycle $1: $(ros2 lifecycle get "$1" 2>&1)"; ros2 topic list -t 2>&1 | head -20' \
        diag "$1" || true
}

# sensor_writes LABEL [--exact] ALLOWED...: what the driver changed on the fake sensor (its
# configuration before and after); fails when a setting outside ALLOWED changed (with --exact:
# unless exactly ALLOWED changed), nothing was configured or something was persisted.
sensor_writes() {
    python3 - "$WORK/$1.fake.json" "$@" <<'PY'
import json, sys
s = json.load(open(sys.argv[1])); label, rest = sys.argv[2], sys.argv[3:]
exact = rest[:1] == ["--exact"]
allowed = set(rest[1:] if exact else rest)
before, after = s["initial"], s["active"]
changed = {k: [before.get(k), after.get(k)] for k in sorted(set(before) | set(after))
           if before.get(k) != after.get(k)}
print(f"{label}: firmware {s['firmware']}, sensor settings changed by the driver: "
      f"{json.dumps(changed, sort_keys=True)}, reinitialized {s['reinits']}x, persisted {s['persisted']}x")
fits = set(changed) == allowed if exact else set(changed) <= allowed
sys.exit(0 if s["reinits"] and fits and not s["persisted"] else 1)
PY
}

stop_and_expect_zero() {  # LABEL
    docker stop -t 15 "$NAME" > /dev/null
    local code
    code=$(docker inspect -f '{{.State.ExitCode}}' "$NAME")
    docker logs "$NAME" > "$WORK/$1.driver.log" 2>&1 || true
    expect "$1: stopped by SIGTERM with exit 0 (exit $code)" test "$code" = 0
    docker rm -f "$NAME" > /dev/null 2>&1 || true
}

sudo ip addr add "$SENSOR_IP/32" dev lo
sudo ip addr add "$HOST_ALIAS/32" dev lo

# ---------------------------------------------------------------------------------------------
echo "== 1. image facts"
facts=$(docker image inspect "$IMAGE" --format '{{json .Config.Entrypoint}} {{json .Config.Cmd}} user={{.Config.User}} workdir={{.Config.WorkingDir}}')
echo "$facts"
[[ "$facts" == "[\"/entrypoint.sh\"] $LEGACY_CMD user= workdir=/opt/ros2_ws" ]] \
    || fail "entrypoint, command, user or working directory changed: $facts"
docker image inspect "$IMAGE" --format '{{range .Config.Env}}{{println .}}{{end}}' | grep -qx 'ROS2_WS=/opt/ros2_ws' \
    || fail "ROS2_WS is not /opt/ros2_ws"

# ---------------------------------------------------------------------------------------------
echo "== 2. legacy mode"
for case in "$REPO"/tests/legacy/cases/*.env; do
    stem=$(basename "$case" .env)
    env_args "$case"
    set +e
    # A command the contract would refuse: in legacy mode the v1.0.0 entrypoint runs it.
    docker run --rm "${ENV_ARGS[@]}" "$IMAGE" bash -c 'echo "legacy: $ROS_DISTRO $AMENT_PREFIX_PATH"' \
        > "$WORK/$stem.out" 2> "$WORK/$stem.err"
    code=$?
    set -e
    if [[ $code == 0 && ! -s "$WORK/$stem.err" ]] && grep -q "^legacy: jazzy .*/opt/ros2_ws/install/ouster_ros" "$WORK/$stem.out"; then
        ok "legacy $stem: the v1.0.0 entrypoint ran the command with ROS and the workspace sourced"
    else
        fail "legacy $stem: exit $code, stdout $(cat "$WORK/$stem.out"), stderr $(cat "$WORK/$stem.err")"
    fi
done

legacy_run() {  # LABEL IMAGE FIRMWARE [check_cloud args]: the fleet compose (no command: the image's CMD)
    local label=$1 image=$2 firmware=$3; shift 3
    start_fake "$label" "$firmware"
    env_args "$REPO/tests/legacy/cases/fleet-compose-default-cmd.env"
    docker run -d --name "$NAME" --network host --ipc host "${ENV_ARGS[@]}" "$image" > /dev/null
    if check_cloud "$label" /ouster/points os_lidar --clouds 3 --timeout 90 "$@"; then
        ok "$label: /ouster/points in frame os_lidar from the recorded packets, every pixel ($*)"
    else
        fail "$label: cloud check failed"
        docker logs "$NAME" 2>&1 | grep -v -E 'fake-sensor' | tail -n 40 || true
    fi
    docker stop -t 15 "$NAME" > /dev/null || true
    docker logs "$NAME" > "$WORK/$label.driver.log" 2>&1 || true
    docker rm -f "$NAME" > /dev/null 2>&1 || true
    stop_fake
    grep -v -E 'fake-sensor: (GET|POST)' "$WORK/$label.fake.log" || true
}
legacy_run legacy "$IMAGE" "$FW_OLD_API" --check-t --imu-topic /ouster/imu --imu-frame os_imu
# The vendor's parameter file (0.15) writes its time-sync defaults to the sensor at every start;
# on a sensor with the factory settings (the fake sensor's) that changes sync_pulse_out_polarity
# (ACTIVE_HIGH -> ACTIVE_LOW). docs/VENDOR_DIFF.md.
expect "legacy: the driver changed only the destination, the ports and sync_pulse_out_polarity on the sensor" \
    sensor_writes legacy --exact udp_dest udp_port_lidar udp_port_imu sync_pulse_out_polarity
grep -E "Will use automatic UDP destination|lidar port set to zero" "$WORK/legacy.driver.log" | head -3 || true
if [[ -n "$OLD_IMAGE" ]]; then
    IMAGE_UNDER_TEST=$IMAGE
    # The released image's SDK (0.11.1) is given the recording's own firmware, 2.3.0.
    legacy_run legacy-old "$OLD_IMAGE" ""
    if python3 - "$WORK/legacy.check.json" "$WORK/legacy-old.check.json" <<'PY'
import json, sys
new, old = (json.load(open(p)) for p in sys.argv[1:3])
# The per-point time t is expected to differ (docs/VENDOR_DIFF.md): compared separately.
T = ("t_measured_column", "t_output_column")
print("t (new, old):", [new.get("pixels", {}).get(k) for k in T], [old.get("pixels", {}).get(k) for k in T])
strip = lambda r: {k: v for k, v in r.items() if k not in ("cloud_list", "pixels", "imu")} | {
    "pixels": {k: v for k, v in r.get("pixels", {}).items() if k not in T}} | {"shape": [
    {k: c[k] for k in ("frame_id", "width", "height", "fields")} for c in r["cloud_list"]]}
a, b = strip(new), strip(old)
print(json.dumps({"new": a, "old": b}, indent=1)[:3000])
sys.exit(0 if a == b else 1)
PY
    then ok "legacy: same clouds as $OLD_IMAGE"; else fail "legacy: clouds differ from $OLD_IMAGE"; fi
    IMAGE=$IMAGE_UNDER_TEST
fi

# ---------------------------------------------------------------------------------------------
echo "== 3. contract: check and refusals in the image equal the host-side goldens"
for case in "$REPO"/tests/regression/cases/*.env; do
    stem=$(basename "$case" .env)
    env_args "$case"
    set +e
    docker run --rm "${ENV_ARGS[@]}" "$IMAGE" check > "$WORK/$stem.out" 2> "$WORK/$stem.err"
    code=$?
    set -e
    {
        printf 'exit %s\n--- stdout\n' "$code"
        cat "$WORK/$stem.out"
        printf -- '--- stderr\n'
        cat "$WORK/$stem.err"
    } > "$WORK/$stem.golden"
    if diff -u "$REPO/tests/regression/golden/$stem.golden" "$WORK/$stem.golden"; then
        ok "contract $stem (exit $code)"
    else
        fail "contract $stem differs from its golden (diff above)"
    fi
done

set +e
docker run --rm -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=os2-128 -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" bash -l \
    > /dev/null 2> "$WORK/cmd.err"
code=$?
set -e
expect "unknown command refused (exit $code)" grep -q "unknown command" "$WORK/cmd.err"
expect "unknown command exits 1" test "$code" = 1

set +e
docker run --rm -e METROLLA_DRIVER_CONTRACT=yes -e LIDAR_MODEL=os2-128 -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" \
    > /dev/null 2> "$WORK/switch.err"
code=$?
set -e
expect "METROLLA_DRIVER_CONTRACT=yes refused with exit 1 (exit $code)" test "$code" = 1
expect "METROLLA_DRIVER_CONTRACT=yes refusal names the switch" grep -q "METROLLA_DRIVER_CONTRACT='yes' is not a contract version" "$WORK/switch.err"

set +e
docker run --rm --read-only --user 97000:97000 -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=os2-128 \
    -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" > /dev/null 2> "$WORK/ro.err"
code=$?
set -e
expect "read-only root without a writable /tmp refused: $(cat "$WORK/ro.err")" grep -q "cannot write" "$WORK/ro.err"
expect "read-only root without a writable /tmp exits 1" test "$code" = 1

# ---------------------------------------------------------------------------------------------
RESTRICT=(--network host --ipc host --user 97000:97000 --read-only --tmpfs /tmp
          --cap-drop ALL --security-opt no-new-privileges:true -e ROS_LOCALHOST_ONLY=1)

echo "== 4a. contract run: defaults (TIME_MODE host, automatic destination), SENSOR_ID east"
CONTRACT_ENV=(-e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=ouster-os2-128 -e "SENSOR_IP=$SENSOR_IP"
              -e SENSOR_ID=east)
start_fake contract "$FW_NEW_API"
docker run -d --name "$NAME" "${RESTRICT[@]}" "${CONTRACT_ENV[@]}" "$IMAGE" > /dev/null
sleep 3
if [[ "$(docker inspect -f '{{.State.Running}}' "$NAME")" != true ]]; then
    fail "driver not running (exit $(docker inspect -f '{{.State.ExitCode}}' "$NAME"))"
    docker logs "$NAME" 2>&1 | tail -n 60
    echo "SMOKE FAILED"
    exit 1
fi
uid=$(docker exec "$NAME" sh -c 'stat -c %u /proc/1')
expect "driver runs as uid $uid (97000)" test "$uid" = 97000
comm=$(docker exec "$NAME" cat /proc/1/comm)
expect "PID 1 is the driver node ($comm)" test "$comm" = os_driver
docker run --rm "${CONTRACT_ENV[@]}" "$IMAGE" check | sed '$d' > "$WORK/expected.yaml"
docker exec "$NAME" cat /tmp/ouster/driver_params.yaml > "$WORK/actual.yaml"
expect "/tmp/ouster/driver_params.yaml in the container equals check's render" \
    diff -u "$WORK/expected.yaml" "$WORK/actual.yaml"
if CHECK_USER=97000:97000 check_cloud contract /east/points east_os_lidar --clouds 3 --timeout 90 --check-t \
        --imu-topic /east/imu --imu-frame east_os_imu; then
    ok "contract: /east/points in frame east_os_lidar, host-time stamps, every pixel and point time; /east/imu in east_os_imu, every value"
else
    fail "contract: cloud check failed"; diagnose /east/os_driver
fi
expect "contract: metadata written to /tmp/ouster/metadata.json" \
    docker exec "$NAME" test -s /tmp/ouster/metadata.json
stop_and_expect_zero contract
stop_fake
grep -v -E 'fake-sensor: (GET|POST)' "$WORK/contract.fake.log" || true
if python3 - "$WORK/contract.fake.json" "$SENSOR_IP" <<'PY'
import json, sys
s = json.load(open(sys.argv[1])); a = s["active"]
print(json.dumps({k: a[k] for k in ("udp_dest", "udp_port_lidar", "udp_port_imu", "timestamp_mode", "operating_mode")}, sort_keys=True), "persisted", s["persisted"], "sent", s["sent"])
sys.exit(0 if (a["udp_port_lidar"], a["udp_port_imu"], a["udp_dest"], a["timestamp_mode"], s["persisted"]) == (7502, 7503, sys.argv[2], "TIME_FROM_INTERNAL_OSC", 0) else 1)
PY
then ok "contract: sensor configured with ports 7502/7503, automatic destination, its own time mode kept, nothing persisted"; else fail "contract: sensor configuration (above)"; fi
expect "contract: nothing but the destination and ports changed on the sensor (time-sync wiring kept)" \
    sensor_writes contract udp_dest udp_port_lidar udp_port_imu

echo "== 4b. contract run: TIME_MODE ptp, HOST_IP, DATA_PORT 7600, absolute POINTCLOUD_TOPIC, FRAME_ID"
CONTRACT_ENV=(-e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=OS-2-128 -e "SENSOR_IP=$SENSOR_IP"
              -e "HOST_IP=$HOST_ALIAS/24" -e DATA_PORT=7600 -e TIME_MODE=ptp -e DEVICE_PORT=80
              -e POINTCLOUD_TOPIC=/lidar/ouster_points -e FRAME_ID=ouster_top -e PUBLISH_SCAN=false
              -e DUAL_LIDAR=false)
start_fake contract-ptp "$FW_WINDOW"
docker run -d --name "$NAME" "${RESTRICT[@]}" "${CONTRACT_ENV[@]}" "$IMAGE" > /dev/null
# The fake sensor stamps packets with UTC; with TIME_FROM_PTP_1588 the driver subtracts the
# 37 s TAI-UTC offset (ptp_utc_tai_offset), so stamps are about 37 s behind the clock here.
if CHECK_USER=97000:97000 check_cloud contract-ptp /lidar/ouster_points ouster_top --clouds 3 --timeout 90 --stamp-window 40 --check-t \
        --imu-topic /ouster/imu --imu-frame os_imu; then
    ok "contract ptp: /lidar/ouster_points in frame ouster_top, every pixel and point time; /ouster/imu"
else
    fail "contract ptp: cloud check failed"; diagnose /ouster/os_driver
fi
if python3 - "$WORK/contract-ptp.check.json" <<'PY'
import json, sys
r = json.load(open(sys.argv[1]))
d = [c["stamp_minus_receive_s"] for c in r["cloud_list"]]
print("stamp - receive:", d)
sys.exit(0 if d and all(-38.5 < x < -36.5 for x in d) else 1)
PY
then ok "contract ptp: stamps are the packets' time minus 37 s"; else fail "contract ptp: stamps not 37 s behind"; fi
stop_and_expect_zero contract-ptp
stop_fake
if python3 - "$WORK/contract-ptp.fake.json" "$HOST_ALIAS" <<'PY'
import json, sys
s = json.load(open(sys.argv[1])); a = s["active"]
print(json.dumps({k: a[k] for k in ("udp_dest", "udp_port_lidar", "udp_port_imu", "timestamp_mode")}, sort_keys=True), "persisted", s["persisted"])
sys.exit(0 if (a["udp_port_lidar"], a["udp_port_imu"], a["udp_dest"], a["timestamp_mode"], s["persisted"]) == (7600, 7601, sys.argv[2], "TIME_FROM_PTP_1588", 0) else 1)
PY
then ok "contract ptp: sensor set to TIME_FROM_PTP_1588, destination HOST_IP, ports 7600/7601"; else fail "contract ptp: sensor configuration (above)"; fi
expect "contract ptp: nothing but the destination, ports and time mode changed on the sensor" \
    sensor_writes contract-ptp udp_dest udp_port_lidar udp_port_imu timestamp_mode

echo "== 4c. contract: SIGTERM while packets arrive, five times (past bug: exit 139)"
start_fake stops "$FW_OLD_API"
for i in 1 2 3 4 5; do
    docker run -d --name "$NAME" "${RESTRICT[@]}" -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=os2-128 \
        -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" > /dev/null
    sleep $((3 + i))
    stop_and_expect_zero "stop-$i"
done
stop_fake

echo "== 5. firmware floor: the SDK (0.16) refuses a sensor on firmware 2.3.0"
start_fake fw-2.3 ""
docker run -d --name "$NAME" "${RESTRICT[@]}" -e METROLLA_DRIVER_CONTRACT=1 -e LIDAR_MODEL=os2-128 \
    -e "SENSOR_IP=$SENSOR_IP" "$IMAGE" > /dev/null
sleep 15
running=$(docker inspect -f '{{.State.Running}}' "$NAME")
docker stop -t 15 "$NAME" > /dev/null || true
code=$(docker inspect -f '{{.State.ExitCode}}' "$NAME")
docker logs "$NAME" > "$WORK/fw-2.3.driver.log" 2>&1 || true
docker rm -f "$NAME" > /dev/null 2>&1 || true
stop_fake
grep -m 3 -E "FW 2.4|Attempting to communicate" "$WORK/fw-2.3.driver.log" || true
echo "fw-2.3: driver was running after 15 s: $running; exit code after docker stop: $code"
expect "fw-2.3: the driver says the sensor needs firmware 2.4 or later" \
    grep -q "Please upgrade your sensor to FW 2.4 or later" "$WORK/fw-2.3.driver.log"
expect "fw-2.3: the sensor was not configured and sent nothing" python3 -c '
import json, sys; s = json.load(open(sys.argv[1])); print("reinits", s["reinits"], "sent", s["sent"])
sys.exit(0 if s["reinits"] == 0 and s["sent"]["lidar"] == 0 else 1)' "$WORK/fw-2.3.fake.json"

if [[ $FAILED != 0 ]]; then
    for log in "$WORK"/*.driver.log; do echo "== $log"; tail -n 60 "$log"; done
    echo "SMOKE FAILED"
    exit 1
fi
echo "SMOKE PASSED"

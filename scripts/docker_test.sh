#!/usr/bin/env bash
# Smoke-test the local :dev image against a headless sim.
# Polls /autodrive/roboracer_1/lap_count and exits PASS as soon as it hits LAP_TARGET,
# or FAIL on TIMEOUT or container exit.
set -euo pipefail

IMAGE="roboracer-icra2026:dev"
SIM_IMAGE="autodriveecosystem/autodrive_roboracer_sim:2026-icra-practice"
LAP_TARGET="${LAP_TARGET:-11}"
TIMEOUT="${TIMEOUT:-1200}"        # 20 min cap (~70s/lap * 11 laps + slack)
POLL_INTERVAL="${POLL_INTERVAL:-5}"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "ERROR: ${IMAGE} not found locally. Run 'pixi run docker_build' first."
  exit 1
fi

cleanup() {
  echo
  echo "Stopping containers..."
  docker rm -f sim-test racer-test >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Starting sim (headless)..."
docker run -d --rm --name sim-test --network=host --ipc=host --privileged \
  --entrypoint /bin/bash "${SIM_IMAGE}" \
  -c "./AutoDRIVE\ Simulator.x86_64 -batchmode -nographics -ip 127.0.0.1 -port 4567" >/dev/null

sleep 5

echo "Starting racer stack..."
docker run -d --rm --name racer-test --network=host --ipc=host \
  --entrypoint /home/autodrive_devkit.sh "${IMAGE}" >/dev/null

echo "Polling /autodrive/roboracer_1/lap_count (target=${LAP_TARGET}, timeout=${TIMEOUT}s, every ${POLL_INTERVAL}s)..."

start=$(date +%s)
last_lap=""

while true; do
  now=$(date +%s)
  elapsed=$((now - start))

  if (( elapsed > TIMEOUT )); then
    echo
    echo "FAIL: timed out after ${elapsed}s. Last seen lap_count: ${last_lap:-<none>}"
    exit 1
  fi

  if ! docker inspect -f '{{.State.Running}}' racer-test 2>/dev/null | grep -q true; then
    echo
    echo "FAIL: racer-test container exited unexpectedly. Last 30 lines:"
    docker logs --tail 30 racer-test 2>&1 || true
    exit 1
  fi
  if ! docker inspect -f '{{.State.Running}}' sim-test 2>/dev/null | grep -q true; then
    echo
    echo "FAIL: sim-test container exited unexpectedly. Last 30 lines:"
    docker logs --tail 30 sim-test 2>&1 || true
    exit 1
  fi

  lap=$(docker exec racer-test bash -c '
    source /opt/ros/humble/setup.bash
    source /home/autodrive_devkit/install/setup.bash
    timeout 3 ros2 topic echo --once /autodrive/roboracer_1/lap_count 2>/dev/null \
      | awk "/^data:/ {print \$2; exit}"
  ' 2>/dev/null || true)

  if [[ "${lap}" =~ ^[0-9]+$ ]]; then
    if [[ "${lap}" != "${last_lap}" ]]; then
      printf "[%4ds] lap_count: %s\n" "${elapsed}" "${lap}"
      last_lap="${lap}"
    fi
    if (( lap >= LAP_TARGET )); then
      echo
      echo "PASS: reached lap_count=${lap} in ${elapsed}s"
      exit 0
    fi
  fi

  sleep "${POLL_INTERVAL}"
done

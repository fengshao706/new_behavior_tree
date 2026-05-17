#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SPEED="${1:-1.0}"

if ! command -v rostopic >/dev/null 2>&1; then
  echo "[ERR] ROS env not sourced. Please source devel/setup.bash first."
  exit 1
fi

if ! rostopic list >/dev/null 2>&1; then
  echo "[INFO] roscore not running, starting one..."
  roscore >/tmp/rm_bt_referee_sim_roscore.log 2>&1 &
  ROSCORE_PID=$!
  sleep 2
else
  ROSCORE_PID=""
fi

cleanup() {
  if [[ -n "${ROSCORE_PID}" ]]; then
    kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "[RUN] referee_simulator.py --speed ${SPEED}"
python3 "${SCRIPT_DIR}/referee_simulator.py" --speed "${SPEED}"

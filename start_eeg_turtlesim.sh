#!/usr/bin/env bash
# ROS 2 setup scripts may read optional variables that are unset. Keep
# nounset disabled while sourcing them, then enable strict mode for our code.
set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${EEG_WORKSPACE_ROOT:-${SCRIPT_DIR}}"
LOG_DIR="${WORKSPACE_ROOT}/logs"
CLASSIFIER_PARAMS_FILE="${WORKSPACE_ROOT}/src/eeg_bci/config/eeg.yaml"
DEVICE_NAME="VIS_BCI_DFED857C"
SAMPLE_RATE="250"
PIDS=()
LAST_PID=""
CLEANED_UP=0

usage() {
  cat <<'EOF'
Usage:
  ./start_eeg_turtlesim.sh              Start the complete EEG+turtlesim demo
  ./start_eeg_turtlesim.sh --no-stimulus Start backend only, without PsychoPy
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

if [[ "${1:-}" != "" && "${1:-}" != "--no-stimulus" ]]; then
  usage >&2
  exit 2
fi

if [[ ! -x "${WORKSPACE_ROOT}/.venv/bin/python" ]]; then
  echo "Missing virtual environment: ${WORKSPACE_ROOT}/.venv" >&2
  echo "Create it with: /usr/bin/python3 -m venv --system-site-packages ${WORKSPACE_ROOT}/.venv" >&2
  exit 1
fi

if [[ ! -f "${CLASSIFIER_PARAMS_FILE}" ]]; then
  echo "Missing classifier parameter file: ${CLASSIFIER_PARAMS_FILE}" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/.venv/bin/activate"
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
mkdir -p "${LOG_DIR}"

cleanup_stale_processes() {
  local pattern pid
  local uid
  uid="$(id -u)"
  echo "Cleaning stale EEG/ROS2 processes..."

  # These patterns are limited to this EEG workspace and the turtlesim/rosbag
  # processes used by this demo. Do not kill unrelated ROS nodes.
  local patterns=(
    "${WORKSPACE_ROOT}/.venv/bin/python -m eeg_bci.ble_to_lsl"
    "${WORKSPACE_ROOT}/.venv/bin/python -m eeg_bci.lsl_to_ros2"
    "${WORKSPACE_ROOT}/.venv/bin/python -m eeg_bci.ssvep_classifier"
    "${WORKSPACE_ROOT}/.venv/bin/python -m eeg_bci.ssvep_stimulus"
    "${WORKSPACE_ROOT}/.venv/bin/python -m eeg_bci.ssvep_to_turtlesim"
    "ros2 run eeg_bci ble_to_lsl"
    "ros2 run eeg_bci lsl_to_ros2"
    "ros2 run eeg_bci ssvep_classifier"
    "ros2 run eeg_bci ssvep_stimulus"
    "ros2 launch eeg_bci turtlesim_ssvep.launch.py"
    "turtlesim_ssvep.launch.py"
    "/opt/ros/humble/lib/turtlesim/turtlesim_node"
    "ros2 bag play"
  )

  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
      echo "  stopping stale PID ${pid}: ${pattern}"
      kill -TERM "${pid}" 2>/dev/null || true
    done < <(pgrep -u "${uid}" -f "${pattern}" || true)
  done

  sleep 1

  for pattern in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "${pid}" || "${pid}" == "$$" || "${pid}" == "${PPID}" ]] && continue
      echo "  force-stopping stale PID ${pid}: ${pattern}"
      kill -KILL "${pid}" 2>/dev/null || true
    done < <(pgrep -u "${uid}" -f "${pattern}" || true)
  done
}

cleanup() {
  [[ "${CLEANED_UP}" == "1" ]] && return
  CLEANED_UP=1
  echo
  echo "Stopping EEG+turtlesim processes..."
  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      # start_background creates a new session, so the PID is also the
      # process-group ID. This stops ros2 wrappers and their Python children.
      kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  sleep 1

  for pid in "${PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  # Also remove matching processes that were started from another terminal,
  # including rosbag playback and turtlesim simulations.
  cleanup_stale_processes
  echo "Stopped. Logs: ${LOG_DIR}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cleanup_stale_processes

start_background() {
  local name="$1"
  shift
  echo "Starting ${name}..."
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  LAST_PID="$!"
  PIDS+=("${LAST_PID}")
}

echo "Starting EEG+turtlesim demo"
echo "Device: ${DEVICE_NAME}; sample rate: ${SAMPLE_RATE} Hz"

start_background "ble_to_lsl" \
  ros2 run eeg_bci ble_to_lsl \
    --device-name "${DEVICE_NAME}" \
    --sample-rate "${SAMPLE_RATE}"

sleep 3

start_background "turtlesim" \
  ros2 launch eeg_bci turtlesim_ssvep.launch.py

sleep 2

start_background "lsl_to_ros2" \
  ros2 run eeg_bci lsl_to_ros2

start_background "ssvep_classifier" \
  ros2 run eeg_bci ssvep_classifier \
    --ros-args \
    --params-file "${CLASSIFIER_PARAMS_FILE}" \
    -p use_trained_model:=false

echo
echo "Backend is running. Logs are in ${LOG_DIR}"
echo "Topic monitor: ros2 topic echo /ssvep/command"

if [[ "${1:-}" == "--no-stimulus" ]]; then
  echo "Stimulus disabled. Press Ctrl+C to stop."
  while true; do sleep 1; done
fi

echo "Starting PsychoPy stimulus; press Ctrl+C or close it to stop."
start_background "ssvep_stimulus" \
  ros2 run eeg_bci ssvep_stimulus
wait "${LAST_PID}" || true

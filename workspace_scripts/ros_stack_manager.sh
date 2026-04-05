#!/usr/bin/env bash

set -euo pipefail

BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$BASE_DIR"

LAUNCH_FILE="$BASE_DIR/src/ROAR_MoveIT/launch/complete.launch.py"
START_DELAY=8
LAUNCH_PID=""
WORKSPACE_PID=""
TELEOP_PID=""
LAUNCH_EXTRA_ARGS=()

usage() {
    cat <<'EOF'
Usage: ./workspace_scripts/ros_stack_manager.sh [options] [-- <extra ros2 launch args>]

Clean up lingering Gazebo, ROS 2, and RViz processes, then launch ROAR_MoveIT complete.launch.py.
After the stack starts, automatically launch workspace.py in the background and the teleop GUI.

Options:
  --delay SECONDS      Delay before starting the optional script. Default: 8
  -h, --help           Show this help

Examples:
  ./workspace_scripts/ros_stack_manager.sh
    ./workspace_scripts/ros_stack_manager.sh --delay 10
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --delay)
            [[ $# -ge 2 ]] || { echo "Missing value for --delay" >&2; exit 1; }
            START_DELAY="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            LAUNCH_EXTRA_ARGS=("$@")
            break
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

OPERATOR_DIR="$BASE_DIR/src/roar_scripts"
WORKSPACE_SCRIPT="$OPERATOR_DIR/workspace.py"
TELEOP_SCRIPT="$OPERATOR_DIR/teleop.py"

if [[ ! -f "$LAUNCH_FILE" ]]; then
    echo "Launch file not found: $LAUNCH_FILE" >&2
    exit 1
fi

if [[ ! -f "$BASE_DIR/install/setup.bash" ]]; then
    echo "Missing workspace setup: $BASE_DIR/install/setup.bash" >&2
    exit 1
fi

[[ -f "$WORKSPACE_SCRIPT" ]] || { echo "Required script not found: $WORKSPACE_SCRIPT" >&2; exit 1; }
[[ -f "$TELEOP_SCRIPT" ]] || { echo "Required script not found: $TELEOP_SCRIPT" >&2; exit 1; }

# Safe defaults for setup files when running with set -u.
export COLCON_TRACE=${COLCON_TRACE:-0}
export AMENT_TRACE=${AMENT_TRACE:-}
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-python3}
export COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_PREFIX_PATH=${COLCON_PREFIX_PATH:-}
export _CATKIN_SETUP_DIR=${_CATKIN_SETUP_DIR:-}
export GZ_VERSION=${GZ_VERSION:-fortress}

# shellcheck disable=SC1091
source "$BASE_DIR/install/setup.bash"

echo "Killing lingering simulator and ROS processes..."
pkill -f gz || true
pkill -f gz_sim || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f ign || true
pkill -f rviz2 || true
pkill -f parameter_bridge || true
pkill -f ros_gz_bridge || true
pkill -f ros_ign_bridge || true
pkill -f ros_gz_sim || true
pkill -f ros_ign_gazebo || true
pkill -f move_group || true
pkill -f robot_state_publisher || true
pkill -f ros2_control_node || true
pkill -f controller_manager || true
pkill -f joint_state_broadcaster || true
pkill -f spawner || true
pkill -f static_transform_publisher || true
pkill -f "ros2 launch $LAUNCH_FILE" || true
pkill -f "$WORKSPACE_SCRIPT" || true
pkill -f "$TELEOP_SCRIPT" || true

sleep 1

cleanup() {
    echo "Cleaning up launcher processes..."
    if [[ -n "${TELEOP_PID:-}" ]]; then
        kill "$TELEOP_PID" 2>/dev/null || true
    fi
    if [[ -n "${WORKSPACE_PID:-}" ]]; then
        kill "$WORKSPACE_PID" 2>/dev/null || true
    fi
    if [[ -n "${LAUNCH_PID:-}" ]]; then
        kill "$LAUNCH_PID" 2>/dev/null || true
    fi
    sleep 1
    pkill -f gz || true
    pkill -f gz_sim || true
    pkill -f gzserver || true
    pkill -f gzclient || true
    pkill -f ign || true
    pkill -f rviz2 || true
    pkill -f parameter_bridge || true
    pkill -f ros_gz_bridge || true
    pkill -f ros_ign_bridge || true
    pkill -f ros_gz_sim || true
    pkill -f ros_ign_gazebo || true
    pkill -f move_group || true
    pkill -f robot_state_publisher || true
    pkill -f ros2_control_node || true
    pkill -f controller_manager || true
    pkill -f joint_state_broadcaster || true
    pkill -f spawner || true
    pkill -f static_transform_publisher || true
    pkill -f "$WORKSPACE_SCRIPT" || true
    pkill -f "$TELEOP_SCRIPT" || true
}

trap cleanup EXIT INT TERM

echo "Starting $LAUNCH_FILE..."
ros2 launch "$LAUNCH_FILE" "${LAUNCH_EXTRA_ARGS[@]}" &
LAUNCH_PID=$!

echo "Waiting $START_DELAY seconds before starting workspace.py and teleop.py..."
sleep "$START_DELAY"

echo "Starting workspace.py in the background..."
python3 "$WORKSPACE_SCRIPT" &
WORKSPACE_PID=$!

echo "Starting teleop GUI..."
python3 "$TELEOP_SCRIPT" &
TELEOP_PID=$!

echo "Started: launch_pid=${LAUNCH_PID:-none} workspace_pid=${WORKSPACE_PID:-none} teleop_pid=${TELEOP_PID:-none}"
echo "Wrapper is running. Close the teleop window or press Ctrl-C to stop everything cleanly."

wait "$TELEOP_PID"
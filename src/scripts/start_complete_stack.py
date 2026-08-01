#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import shutil
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
VALID_TIP_VARIANTS = {"sampling", "probe", "maintenence"}

KILL_PATTERNS = [
    "ros2 launch sixdof_moveit complete.launch.py",
    "ros2 launch sixdof_pkg gazebo.launch.py",
    "python3 src/scripts/teleop.py",
    "gripper_joint_state_republisher",
    # Keep ee_ref single-owned: a stale broadcaster will keep publishing the
    # old frame pose and fight with the current teleop-controlled instance.
    "reference_frame_broadcaster.py",
    "static_world_to_base.py",
    "python3 src/scripts/workspace.py",
    "python3 src/scripts/workspace_checker.py",
    "move_group",
    "rviz2",
    "robot_state_publisher",
    "ros_gz_bridge",
    "spawner",
    "ign gazebo",
]

CHILDREN: list[tuple[str, subprocess.Popen]] = []
RVIZ_SOURCE = WORKSPACE_ROOT / "src" / "sixdof_moveit" / "config" / "moveit.rviz"
RVIZ_INSTALL = WORKSPACE_ROOT / "install" / "sixdof_moveit" / "share" / "sixdof_moveit" / "config" / "moveit.rviz"


def run_shell(command: str) -> int:
    return subprocess.run(["bash", "-lc", command], check=False).returncode


def build_env_prefix(tip_variant: str) -> str:
    return (
        "unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH && "
        f"cd {shlex.quote(str(WORKSPACE_ROOT))} && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        f"export SIXDOF_TIP_VARIANT={shlex.quote(tip_variant)}"
    )


def run_env_shell(env_prefix: str, command: str) -> int:
    return run_shell(f"{env_prefix} && {command}")


def wait_for_env_shell(env_prefix: str, command: str, description: str, timeout_sec: float, poll_sec: float = 0.5) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if run_env_shell(env_prefix, command) == 0:
            print(f"[ready] {description}")
            return True
        time.sleep(poll_sec)
    print(f"[warn] timed out waiting for {description}")
    return False


def kill_lingering_processes() -> None:
    for pattern in KILL_PATTERNS:
        run_shell(f"pkill -f {shlex.quote(pattern)} || true")
    time.sleep(1.0)


def sync_rviz_config() -> None:
    if not RVIZ_SOURCE.exists():
        print(f"RViz source config not found: {RVIZ_SOURCE}")
        return
    RVIZ_INSTALL.parent.mkdir(parents=True, exist_ok=True)
    try:
        if RVIZ_INSTALL.exists() and RVIZ_SOURCE.samefile(RVIZ_INSTALL):
            print(f"[sync] rviz config already current: {RVIZ_INSTALL}")
            return
    except FileNotFoundError:
        pass
    shutil.copy2(RVIZ_SOURCE, RVIZ_INSTALL)
    print(f"[sync] rviz config: {RVIZ_SOURCE} -> {RVIZ_INSTALL}")


def spawn_process(env_prefix: str, name: str, command: str) -> subprocess.Popen:
    proc = subprocess.Popen(
        ["bash", "-lc", f"{env_prefix} && {command}"],
        preexec_fn=os.setsid,
    )
    CHILDREN.append((name, proc))
    print(f"[start] {name}: pid={proc.pid}")
    return proc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start the full sixdof stack with an optional tool-tip variant.")
    parser.add_argument(
        "--tip-variant",
        default=os.environ.get("SIXDOF_TIP_VARIANT", "sampling"),
        choices=sorted(VALID_TIP_VARIANTS),
        help="Tool-tip variant to load for Gazebo, MoveIt, teleop, and workspace tools.",
    )
    return parser.parse_args()


def stop_children() -> None:
    for _, proc in reversed(CHILDREN):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    for _, proc in reversed(CHILDREN):
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def main() -> int:
    args = parse_args()
    env_prefix = build_env_prefix(args.tip_variant)

    print(f"Workspace: {WORKSPACE_ROOT}")
    print(f"Tip variant: {args.tip_variant}")
    print("Killing lingering ROS, Gazebo, RViz, MoveIt, and script processes...")
    kill_lingering_processes()
    sync_rviz_config()

    try:
        complete = spawn_process(env_prefix, "complete_launch", "ros2 launch sixdof_moveit complete.launch.py")
        time.sleep(12.0)
        if complete.poll() is not None:
            print("complete.launch.py exited early")
            return complete.returncode or 1

        spawn_process(env_prefix, "teleop", "python3 src/scripts/teleop.py")
        time.sleep(1.0)
        # Ensure a 'world' -> 'base_link' static frame exists for TF lookups
        spawn_process(env_prefix, "static_tf", "python3 src/scripts/static_world_to_base.py")
        time.sleep(0.5)
        # Start the only ee_ref broadcaster after cleanup so teleop updates go
        # to a single TF authority for the lifetime of this stack.
        spawn_process(env_prefix, "ref_broadcaster", "python3 src/scripts/reference_frame_broadcaster.py")
        wait_for_env_shell(
            env_prefix,
            "ros2 service type /compute_fk >/dev/null 2>&1",
            "/compute_fk service",
            timeout_sec=25.0,
        )
        spawn_process(env_prefix, "workspace", "python3 src/scripts/workspace.py")
        time.sleep(1.0)
        run_env_shell(
            env_prefix,
            "ros2 action send_goal /hand_controller_controller/follow_joint_trajectory "
            "control_msgs/action/FollowJointTrajectory "
            "'{trajectory: {joint_names: [left_gripper, right_gripper], points: [{positions: [0.069, 0.0], time_from_start: {sec: 1}}]}}'"
        )

        print("All processes started. Press Ctrl+C to stop everything.")
        while True:
            time.sleep(1.0)
            if complete.poll() is not None:
                print("complete.launch.py exited; stopping child processes.")
                return complete.returncode or 1
    except KeyboardInterrupt:
        print("Stopping all launched processes...")
        return 0
    finally:
        stop_children()


if __name__ == "__main__":
    sys.exit(main())
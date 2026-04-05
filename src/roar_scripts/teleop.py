#!/usr/bin/env python3
"""
teleop.py — GUI teleop: world-frame XYZ + joint-space rotation + orientation lock
"""

import argparse
import json
import math
import threading
from queue import Empty, Queue
import tkinter as tk
from tkinter import scrolledtext

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

LINK_NAME = "Link_5"
GROUP_NAME = "arm_controller"
FRAME_ID = "world"
BASE_FRAME = "base_link"
DEFAULT_CM = 1.0
DEFAULT_DEG = 10.0
PITCH_JOINT = "Joint_4"
TWIST_JOINT = "Joint_5"
LOCK_TOL = 0.05
ROT_JOINT_TOL = 0.03

JOINT_LIMITS = {
    "Joint_1": (-1.5808, 1.5808),
    "Joint_2": (-2.4535, 0.01),
    "Joint_3": (-0.1, 3.1416),
    "Joint_4": (-1.5808, 1.5808),
    "Joint_5": (-1.5808, 1.5808),
}

HOME_JOINTS = {
    "Joint_1": math.radians(30.0),
    "Joint_2": math.radians(-40.0),
    "Joint_3": math.radians(160.0),
    "Joint_4": math.radians(70.0),
    "Joint_5": math.radians(0.0),
}

ROT_AXIS_TO_JOINT = {"z": "Joint_1", "y": "Joint_2", "x": "Joint_3"}


class Teleop(Node):
    def __init__(self, log_callback=None):
        super().__init__("roar_teleop")
        self._client = ActionClient(self, MoveGroup, "move_action")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = threading.Event()
        self.done.set()
        self.joints = {}
        self.log_callback = log_callback
        self.create_subscription(JointState, "joint_states", self._js, 10)
        self.locked = False
        self.locked_pitch = 0.0
        self.locked_twist = 0.0
        lock_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lock_pub = self.create_publisher(String, "/roar/orientation_lock", lock_qos)
        self._publish_lock_state()

    def _log(self, level, text):
        logger = self.get_logger()
        if level == "error":
            logger.error(text)
        elif level == "warn":
            logger.warn(text)
        else:
            logger.info(text)
        if self.log_callback is not None:
            self.log_callback(level, text)

    def _publish_lock_state(self):
        msg = String()
        msg.data = json.dumps(
            {
                "locked": self.locked,
                "joint_4": float(self.locked_pitch),
                "joint_5": float(self.locked_twist),
            }
        )
        self.lock_pub.publish(msg)

    def _js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    def _tf(self, parent, child):
        try:
            transform = self.tf_buffer.lookup_transform(
                parent,
                child,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return transform.transform.translation
        except Exception as exc:
            self._log("error", f"TF {parent}->{child}: {exc}")
            return None

    def lock(self):
        missing = [joint for joint in [PITCH_JOINT, TWIST_JOINT] if joint not in self.joints]
        if missing:
            self._log("error", f"No joint state for: {missing}")
            return
        self.locked_pitch = self.joints[PITCH_JOINT]
        self.locked_twist = self.joints[TWIST_JOINT]
        self.locked = True
        self._log(
            "info",
            f"LOCKED - {PITCH_JOINT}={math.degrees(self.locked_pitch):.1f} deg  "
            f"{TWIST_JOINT}={math.degrees(self.locked_twist):.1f} deg",
        )
        self._publish_lock_state()

    def unlock(self):
        self.locked = False
        self._log("info", "UNLOCKED")
        self._publish_lock_state()

    def status_text(self):
        lock_str = (
            f"LOCKED  J4={math.degrees(self.locked_pitch):.1f} deg  "
            f"J5={math.degrees(self.locked_twist):.1f} deg"
            if self.locked
            else "free"
        )
        lines = ["Status", f"Lock: {lock_str}", "Joints:"]
        for name in ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]:
            value = self.joints.get(name, float("nan"))
            lines.append(f"  {name}  {value:+.4f} rad  ({math.degrees(value):+.1f} deg)")
        world_pos = self._tf(FRAME_ID, LINK_NAME)
        if world_pos:
            lines.append(f"EE world: x={world_pos.x:.4f} y={world_pos.y:.4f} z={world_pos.z:.4f}")
        base_pos = self._tf(BASE_FRAME, LINK_NAME)
        if base_pos:
            dist = math.sqrt(base_pos.x ** 2 + base_pos.y ** 2 + base_pos.z ** 2)
            lines.append(
                f"EE base:  x={base_pos.x:.4f} y={base_pos.y:.4f} z={base_pos.z:.4f} dist={dist:.4f}m"
            )
        return "\n".join(lines)

    def print_status(self):
        self._log("info", self.status_text())

    def _lock_jc(self):
        if not self.locked:
            return []
        output = []
        for joint_name, joint_value in [(PITCH_JOINT, self.locked_pitch), (TWIST_JOINT, self.locked_twist)]:
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_value
            joint_constraint.tolerance_above = LOCK_TOL
            joint_constraint.tolerance_below = LOCK_TOL
            joint_constraint.weight = 1.0
            output.append(joint_constraint)
        return output

    def _joint_constraint(self, joint_name, position, tol=ROT_JOINT_TOL):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint_name
        joint_constraint.position = position
        joint_constraint.tolerance_above = tol
        joint_constraint.tolerance_below = tol
        joint_constraint.weight = 1.0
        return joint_constraint

    def _clamp_joint(self, joint_name, value):
        lower, upper = JOINT_LIMITS[joint_name]
        return max(lower, min(upper, value))

    def _pos_constraint(self, x, y, z, tol=0.01):
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = FRAME_ID
        position_constraint.link_name = LINK_NAME
        position_constraint.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tol, tol, tol]
        position_constraint.constraint_region.primitives.append(box)
        target_pose = PoseStamped()
        target_pose.header.frame_id = FRAME_ID
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        return position_constraint

    def move_xyz(self, dx, dy, dz, label):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        self.done.clear()
        position = self._tf(FRAME_ID, LINK_NAME)
        if position is None:
            self.done.set()
            return
        constraints = Constraints()
        constraints.position_constraints.append(self._pos_constraint(position.x + dx, position.y + dy, position.z + dz))
        constraints.joint_constraints.extend(self._lock_jc())
        cm = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) * 100.0
        lock_note = " [LOCKED]" if self.locked else ""
        self._log("info", f"XYZ {cm:.1f}cm {label}{lock_note}")
        self._send(constraints)

    def go_home(self):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        self.done.clear()
        constraints = Constraints()
        for joint_name in ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]:
            constraints.joint_constraints.append(self._joint_constraint(joint_name, HOME_JOINTS[joint_name], tol=0.03))
        self._log("info", "HOME -> J1=30 deg, J2=-40 deg, J3=160 deg, J4=70 deg, J5=0 deg")
        self._send(constraints)

    def rotate_joint_axis(self, axis, angle_rad, label):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        self.done.clear()
        target_joint = ROT_AXIS_TO_JOINT[axis]
        if any(joint_name not in self.joints for joint_name in ["Joint_1", "Joint_2", "Joint_3"]):
            self._log("error", "Missing joint states for Joint_1/Joint_2/Joint_3")
            self.done.set()
            return
        target = self._clamp_joint(target_joint, self.joints[target_joint] + angle_rad)
        if abs(target - self.joints[target_joint]) < 1e-6:
            self._log("warn", f"{target_joint} already at limit")
            self.done.set()
            return
        constraints = Constraints()
        for joint_name in ["Joint_1", "Joint_2", "Joint_3"]:
            position = target if joint_name == target_joint else self.joints[joint_name]
            constraints.joint_constraints.append(self._joint_constraint(joint_name, position))
        constraints.joint_constraints.extend(self._lock_jc())
        lock_note = " [LOCKED]" if self.locked else ""
        self._log(
            "info",
            f"ROT {label} {math.degrees(angle_rad):+.1f} deg -> {target_joint} {math.degrees(target):+.1f} deg{lock_note}",
        )
        self._send(constraints)

    def _send(self, constraints):
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.allowed_planning_time = 3.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.goal_constraints.append(constraints)
        self._client.wait_for_server()
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._log("warn", "Goal rejected.")
            self.done.set()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        value = future.result().result.error_code.val
        self._log("info", "Done." if value == 1 else f"Failed (code {value})")
        self.done.set()


class TeleopGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.log_queue = Queue()
        self.xyz_step_var = tk.StringVar(value=str(DEFAULT_CM))
        self.rot_step_var = tk.StringVar(value=str(DEFAULT_DEG))
        self.lock_var = tk.StringVar(value="Lock: free")
        self.exec_var = tk.StringVar(value="Planner: idle")
        self.joint_var = tk.StringVar(value="Waiting for joint states...")

        self.node.log_callback = self.enqueue_log

        self.root.title("ROAR Teleop GUI")
        self.root.geometry("920x720")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_ui()
        self.root.after(100, self._drain_logs)
        self.root.after(250, self._refresh_status)

    def enqueue_log(self, level, text):
        self.log_queue.put((level, text))

    def _build_ui(self):
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        header = tk.Label(main, text="ROAR Arm Teleop", font=("TkDefaultFont", 16, "bold"))
        header.pack(anchor=tk.W)

        info = tk.Label(
            main,
            text="Launches alongside Gazebo and RViz. XYZ moves are world-frame. Rotation controls joints 1-3.",
            justify=tk.LEFT,
        )
        info.pack(anchor=tk.W, pady=(4, 10))

        status = tk.Frame(main)
        status.pack(fill=tk.X, pady=(0, 12))
        tk.Label(status, textvariable=self.lock_var, width=32, anchor=tk.W).pack(side=tk.LEFT)
        tk.Label(status, textvariable=self.exec_var, width=18, anchor=tk.W).pack(side=tk.LEFT, padx=(12, 0))

        controls = tk.Frame(main)
        controls.pack(fill=tk.X)

        xyz = tk.LabelFrame(controls, text="XYZ Move (cm)", padx=10, pady=10)
        xyz.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        tk.Label(xyz, text="Step").grid(row=0, column=0, sticky="w")
        tk.Entry(xyz, textvariable=self.xyz_step_var, width=8).grid(row=0, column=1, sticky="w")
        tk.Button(xyz, text="+X", width=8, command=lambda: self._move_xyz(+1, 0, 0, "+X forward")).grid(row=1, column=1, pady=4)
        tk.Button(xyz, text="-X", width=8, command=lambda: self._move_xyz(-1, 0, 0, "-X back")).grid(row=3, column=1, pady=4)
        tk.Button(xyz, text="+Y", width=8, command=lambda: self._move_xyz(0, +1, 0, "+Y left")).grid(row=2, column=0, padx=4)
        tk.Button(xyz, text="-Y", width=8, command=lambda: self._move_xyz(0, -1, 0, "-Y right")).grid(row=2, column=2, padx=4)
        tk.Button(xyz, text="+Z", width=8, command=lambda: self._move_xyz(0, 0, +1, "+Z up")).grid(row=1, column=3, padx=(12, 0))
        tk.Button(xyz, text="-Z", width=8, command=lambda: self._move_xyz(0, 0, -1, "-Z down")).grid(row=3, column=3, padx=(12, 0))

        rot = tk.LabelFrame(controls, text="Rotation (deg)", padx=10, pady=10)
        rot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))
        tk.Label(rot, text="Step").grid(row=0, column=0, sticky="w")
        tk.Entry(rot, textvariable=self.rot_step_var, width=8).grid(row=0, column=1, sticky="w")
        tk.Button(rot, text="Joint_1 +", width=12, command=lambda: self._rotate("z", +1, "Joint_1 yaw +")).grid(row=1, column=0, pady=4)
        tk.Button(rot, text="Joint_1 -", width=12, command=lambda: self._rotate("z", -1, "Joint_1 yaw -")).grid(row=1, column=1, pady=4)
        tk.Button(rot, text="Joint_2 +", width=12, command=lambda: self._rotate("y", +1, "Joint_2 shoulder +")).grid(row=2, column=0, pady=4)
        tk.Button(rot, text="Joint_2 -", width=12, command=lambda: self._rotate("y", -1, "Joint_2 shoulder -")).grid(row=2, column=1, pady=4)
        tk.Button(rot, text="Joint_3 +", width=12, command=lambda: self._rotate("x", +1, "Joint_3 elbow +")).grid(row=3, column=0, pady=4)
        tk.Button(rot, text="Joint_3 -", width=12, command=lambda: self._rotate("x", -1, "Joint_3 elbow -")).grid(row=3, column=1, pady=4)

        actions = tk.LabelFrame(main, text="Actions", padx=10, pady=10)
        actions.pack(fill=tk.X, pady=(12, 12))
        tk.Button(actions, text="Home", width=12, command=self.node.go_home).pack(side=tk.LEFT)
        tk.Button(actions, text="Lock J4/J5", width=12, command=self.node.lock).pack(side=tk.LEFT, padx=6)
        tk.Button(actions, text="Unlock", width=12, command=self.node.unlock).pack(side=tk.LEFT, padx=6)
        tk.Button(actions, text="Refresh Status", width=14, command=self.node.print_status).pack(side=tk.LEFT, padx=6)
        tk.Button(actions, text="Quit", width=12, command=self.close).pack(side=tk.RIGHT)

        joint_frame = tk.LabelFrame(main, text="Current State", padx=10, pady=10)
        joint_frame.pack(fill=tk.X, pady=(0, 12))
        tk.Label(joint_frame, textvariable=self.joint_var, justify=tk.LEFT, anchor="w").pack(fill=tk.X)

        log_frame = tk.LabelFrame(main, text="Log", padx=10, pady=10)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_box = scrolledtext.ScrolledText(log_frame, height=18, state=tk.DISABLED, wrap=tk.WORD)
        self.log_box.pack(fill=tk.BOTH, expand=True)

    def _append_log(self, line):
        self.log_box.configure(state=tk.NORMAL)
        self.log_box.insert(tk.END, line + "\n")
        self.log_box.see(tk.END)
        self.log_box.configure(state=tk.DISABLED)

    def _drain_logs(self):
        try:
            while True:
                level, text = self.log_queue.get_nowait()
                self._append_log(f"[{level.upper()}] {text}")
        except Empty:
            pass
        self.root.after(100, self._drain_logs)

    def _refresh_status(self):
        if self.node.locked:
            self.lock_var.set(
                f"Lock: locked  J4={math.degrees(self.node.locked_pitch):.1f} deg  "
                f"J5={math.degrees(self.node.locked_twist):.1f} deg"
            )
        else:
            self.lock_var.set("Lock: free")
        self.exec_var.set("Planner: busy" if not self.node.done.is_set() else "Planner: idle")
        values = []
        for name in ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]:
            if name in self.node.joints:
                values.append(f"{name}: {math.degrees(self.node.joints[name]):+.1f} deg")
            else:
                values.append(f"{name}: n/a")
        self.joint_var.set("\n".join(values))
        self.root.after(250, self._refresh_status)

    def _parse_float(self, value, label):
        try:
            parsed = float(value)
        except ValueError:
            self.enqueue_log("error", f"Invalid {label}: {value}")
            return None
        return parsed

    def _move_xyz(self, x_sign, y_sign, z_sign, label):
        cm = self._parse_float(self.xyz_step_var.get(), "XYZ step")
        if cm is None:
            return
        metres = cm / 100.0
        self.node.move_xyz(x_sign * metres, y_sign * metres, z_sign * metres, label)

    def _rotate(self, axis, sign, label):
        deg = self._parse_float(self.rot_step_var.get(), "rotation step")
        if deg is None:
            return
        self.node.rotate_joint_axis(axis, sign * math.radians(deg), label)

    def close(self):
        self.root.quit()


def run_headless_check():
    rclpy.init()
    node = Teleop()
    node._log("info", "Headless teleop check passed.")
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless-check", action="store_true", help="Initialize the ROS node without starting the GUI")
    args = parser.parse_args()

    if args.headless_check:
        run_headless_check()
        return

    rclpy.init()
    node = Teleop()
    root = tk.Tk()
    app = TeleopGui(root, node)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        app._append_log("[INFO] Teleop GUI ready.")
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
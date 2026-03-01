#!/usr/bin/env python3
"""
teleop.py — Full teleop: world-frame XYZ + joint-space rotation + orientation lock
-----------------------------------------------------------------------------------
kinematics.yaml: KDL, position_only_ik: true  (unchanged)

HOW ROTATION WORKS:
    Rotation commands are joint-space and intuitive:
        rz/rz- -> Joint_1 (base yaw)
        ry/ry- -> Joint_2 (shoulder pitch)
        rx/rx- -> Joint_3 (elbow pitch)
    This gives direct "human-style" control of joints 1-3.

ORIENTATION LOCK:
  Locks Joint_4 (pitch) + Joint_5 (twist) as joint constraints.
  Automatically applied to every move command while locked.

COMMANDS:
  XYZ (cm):       w/s/a/d/q/e [cm]
  Rotation (deg): rz/rz-/ry/ry-/rx/rx- [deg]
    Home:           h   go to saved HOME pose
  Lock:           c   lock current J4+J5
  Unlock:         u
  Info:           p   print joints + lock status
  Quit:           x

ROS TOPICS (GUI-compatible):
    /teleop_command (std_msgs/String): same text commands as CLI (e.g. "w 5", "rz 10", "h", "c")
    /lock_orientation (std_msgs/String): true/false or lock/unlock
    /ik_target_pose (std_msgs/Float64MultiArray): [x,y,z,yaw,pitch,roll] from GUI IK mode
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String, Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from collision_guard import CollisionGuard
import sys, termios, threading, math
import json
import os
import xml.etree.ElementTree as ET

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

LINK_NAME   = "ee_link"
GROUP_NAME  = "arm_controller"
FRAME_ID    = "world"
BASE_FRAME  = "base_link"
DEFAULT_CM  = 1.0
DEFAULT_DEG = 10.0
PITCH_JOINT = "Joint_4"
TWIST_JOINT = "Joint_5"
LOCK_TOL    = 0.05
ROT_JOINT_TOL = 0.03

JOINT_LIMITS = {
    "Joint_1": (-0.0170,  3.1590),
    "Joint_2": (-2.6350,  0.0170),
    "Joint_3": (-0.0170,  3.1590),
    "Joint_4": (-0.0170,  3.1590),
    "Joint_5": (-0.0170,  3.1590),
}

# Safe HOME pose within rover URDF/SRDF bounds (degrees -> radians)
HOME_JOINTS = {
    "Joint_1": math.radians(0.0),
    "Joint_2": math.radians(0.0),
    "Joint_3": math.radians(0.0),
    "Joint_4": math.radians(0.0),
    "Joint_5": math.radians(0.0),
}
ARM_JOINTS = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]

DIRECTION_MAP = {
    "w": (+1,  0,  0), "s": (-1,  0,  0),
    "a": ( 0, +1,  0), "d": ( 0, -1,  0),
    "q": ( 0,  0, +1), "e": ( 0,  0, -1),
}
AXIS_LABEL = {
    "w": "+X fwd", "s": "-X back",
    "a": "+Y left", "d": "-Y right",
    "q": "+Z up",   "e": "-Z down",
}
ROT_CMDS = {
    "rz":  ("z", +1, "Joint_1 + (base yaw)"),
    "rz-": ("z", -1, "Joint_1 - (base yaw)"),
    "ry":  ("y", +1, "Joint_2 + (shoulder)"),
    "ry-": ("y", -1, "Joint_2 - (shoulder)"),
    "rx":  ("x", +1, "Joint_3 + (elbow)"),
    "rx-": ("x", -1, "Joint_3 - (elbow)"),
}
ROT_AXIS_TO_JOINT = {"z": "Joint_1", "y": "Joint_2", "x": "Joint_3"}

msg = f"""
┌────────────────────────────────────────────────────────┐
│                  ROVER Full Teleop                     │
│  World-frame XYZ + Joint Rotation + Orientation Lock   │
├────────────────────────────────────────────────────────┤
│  XYZ (world frame, cm):                                │
│    w/s → world X   a/d → world Y   q/e → world Z       │
│    "w 5" = 5cm,  no number = {DEFAULT_CM:.0f}cm default│
│                                                        │
│  ROTATION (joint-space, human-style):                  │
│    rz  / rz-  → Joint_1 yaw + / -                      │
│    ry  / ry-  → Joint_2 shoulder + / -                 │
│    rx  / rx-  → Joint_3 elbow + / -                    │
│    "rz 10" = move that joint by 10°                    │
│                                                        │
│  ORIENTATION LOCK (J4 pitch + J5 twist):               │
│    c  → lock current orientation                       │
│    u  → unlock                                         │
│                                                        │
│  HOME MACRO:                                           │
│    h  → move to saved HOME joint pose                  │
│                                                        │
│    p  → print joints + lock status + EE position       │
│    x  → quit                                           │
└────────────────────────────────────────────────────────┘
"""


def _resolve_urdf_path() -> str | None:
    if get_package_share_directory is not None:
        try:
            return os.path.join(get_package_share_directory("legacy_pkg"), "urdf", "legacy.urdf")
        except Exception:
            pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "legacy_pkg", "urdf", "legacy.urdf"))


def _load_arm_joint_limits_from_urdf(base_link: str, ee_link: str) -> tuple[list[str], dict[str, tuple[float, float]]]:
    urdf_path = _resolve_urdf_path()
    if not urdf_path or not os.path.exists(urdf_path):
        return ARM_JOINTS, JOINT_LIMITS.copy()

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return ARM_JOINTS, JOINT_LIMITS.copy()

    joints_by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            joints_by_child[child.get("link")] = joint

    chain = []
    link = ee_link
    while link != base_link and link in joints_by_child:
        joint = joints_by_child[link]
        chain.append(joint)
        parent = joint.find("parent")
        if parent is None or not parent.get("link"):
            break
        link = parent.get("link")

    chain.reverse()
    joint_names = []
    limits: dict[str, tuple[float, float]] = {}
    for joint in chain:
        j_type = (joint.get("type") or "").lower()
        if j_type not in ("revolute", "prismatic", "continuous"):
            continue

        j_name = joint.get("name")
        if not j_name:
            continue

        if j_type == "continuous":
            lo, hi = -math.pi, math.pi
        else:
            lim = joint.find("limit")
            if lim is None:
                continue
            lo = float(lim.get("lower", "0.0"))
            hi = float(lim.get("upper", "0.0"))

        joint_names.append(j_name)
        limits[j_name] = (lo, hi)

    if len(joint_names) < 3:
        return ARM_JOINTS, JOINT_LIMITS.copy()
    return joint_names, limits


class Teleop(Node):
    def __init__(self):
        super().__init__("rover_teleop")
        self._client = ActionClient(self, MoveGroup, "move_action")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = threading.Event()
        self.done.set()
        self.joints: dict[str, float] = {}
        self.arm_joint_names, self.joint_limits = _load_arm_joint_limits_from_urdf(BASE_FRAME, LINK_NAME)
        self.home_joints = {
            jn: max(self.joint_limits[jn][0], min(self.joint_limits[jn][1], HOME_JOINTS.get(jn, 0.0)))
            for jn in self.arm_joint_names
        }
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
        self.lock_pub = self.create_publisher(String, "/rover/orientation_lock", lock_qos)
        self.create_subscription(String, "/teleop_command", self._teleop_cmd_cb, 10)
        self.create_subscription(String, "/lock_orientation", self._lock_cmd_cb, 10)
        self.create_subscription(Float64MultiArray, "/ik_target_pose", self._ik_target_cb, 10)
        self.collision_guard = CollisionGuard(self, GROUP_NAME)
        self.get_logger().info(
            f"Using group='{GROUP_NAME}', joints={self.arm_joint_names}, urdf_limits_loaded={bool(self.arm_joint_names)}"
        )
        self.get_logger().info("Listening: /teleop_command, /lock_orientation, /ik_target_pose")
        self._publish_lock_state()

    def _teleop_cmd_cb(self, msg: String):
        line = (msg.data or "").strip()
        if not line:
            return
        parsed = parse(line)
        if parsed is None:
            return
        self._execute(parsed, source="ros")

    def _lock_cmd_cb(self, msg: String):
        text = (msg.data or "").strip().lower()
        if text in ("true", "1", "lock", "locked", "on"):
            self.lock()
        elif text in ("false", "0", "unlock", "unlocked", "off"):
            self.unlock()

    def _ik_target_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn("/ik_target_pose requires at least [x,y,z]")
            return

        x_raw, y_raw, z_raw = float(data[0]), float(data[1]), float(data[2])

        max_abs = max(abs(x_raw), abs(y_raw), abs(z_raw))
        if max_abs > 3.0:
            x, y, z = x_raw / 100.0, y_raw / 100.0, z_raw / 100.0
            unit = "cm"
        else:
            x, y, z = x_raw, y_raw, z_raw
            unit = "m"

        if not self.done.is_set():
            self.get_logger().warn("IK target ignored: still executing previous command.")
            return

        self.get_logger().info(
            f"IK target ({unit}) -> world x={x:.3f}, y={y:.3f}, z={z:.3f}"
            + ("  [LOCKED]" if self.locked else "")
        )
        self.move_xyz_absolute(x, y, z)

    def _publish_lock_state(self):
        msg = String()
        msg.data = json.dumps({
            "locked": self.locked,
            "joint_4": float(self.locked_pitch),
            "joint_5": float(self.locked_twist),
        })
        self.lock_pub.publish(msg)

    def _js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def _tf(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return t.transform.translation
        except Exception as e:
            self.get_logger().error(f"TF {parent}→{child}: {e}")
            return None

    # --- Lock ---
    def lock(self):
        missing = [j for j in [PITCH_JOINT, TWIST_JOINT] if j not in self.joints]
        if missing:
            self.get_logger().error(f"No joint state for: {missing}")
            return
        self.locked_pitch = self.joints[PITCH_JOINT]
        self.locked_twist = self.joints[TWIST_JOINT]
        self.locked = True
        self.get_logger().info(
            f"LOCKED — {PITCH_JOINT}={math.degrees(self.locked_pitch):.1f}°  "
            f"{TWIST_JOINT}={math.degrees(self.locked_twist):.1f}°")
        self._publish_lock_state()

    def unlock(self):
        self.locked = False
        self.get_logger().info("UNLOCKED")
        self._publish_lock_state()

    def print_status(self):
        lock_str = (f"LOCKED  J4={math.degrees(self.locked_pitch):.1f}°  "
                    f"J5={math.degrees(self.locked_twist):.1f}°"
                    if self.locked else "free")
        lines = ["", "═══ Status ═══", f"  Lock: {lock_str}", "  Joints:"]
        for n in self.arm_joint_names:
            v = self.joints.get(n, float("nan"))
            lines.append(f"    {n}  {v:+.4f} rad  ({math.degrees(v):+.1f}°)")
        p = self._tf(FRAME_ID, LINK_NAME)
        if p:
            lines.append(f"  EE (world): x={p.x:.4f}  y={p.y:.4f}  z={p.z:.4f}")
        p2 = self._tf(BASE_FRAME, LINK_NAME)
        if p2:
            dist = math.sqrt(p2.x**2 + p2.y**2 + p2.z**2)
            lines.append(f"  EE (base):  x={p2.x:.4f}  y={p2.y:.4f}  z={p2.z:.4f}  dist={dist:.4f}m")
        lines.append("══════════════")
        self.get_logger().info("\n".join(lines))

    def _lock_jc(self):
        if not self.locked:
            return []
        out = []
        for jn, jv in [(PITCH_JOINT, self.locked_pitch), (TWIST_JOINT, self.locked_twist)]:
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = jv
            jc.tolerance_above = LOCK_TOL
            jc.tolerance_below = LOCK_TOL
            jc.weight = 1.0
            out.append(jc)
        return out

    def _joint_constraint(self, joint_name: str, position: float, tol: float = ROT_JOINT_TOL):
        jc = JointConstraint()
        jc.joint_name = joint_name
        jc.position = position
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        return jc

    def _clamp_joint(self, joint_name: str, value: float):
        lo, hi = self.joint_limits[joint_name]
        return max(lo, min(hi, value))

    def _pos_constraint(self, x, y, z, tol=0.01):
        pc = PositionConstraint()
        pc.header.frame_id = FRAME_ID
        pc.link_name = LINK_NAME
        pc.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tol, tol, tol]
        pc.constraint_region.primitives.append(box)
        tp = PoseStamped()
        tp.header.frame_id = FRAME_ID
        tp.pose.position.x = x
        tp.pose.position.y = y
        tp.pose.position.z = z
        tp.pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(tp.pose)
        return pc

    # --- XYZ move ---
    def move_xyz(self, dx, dy, dz):
        if not self.done.is_set():
            self.get_logger().warn("Still executing.")
            return
        self.done.clear()
        pos = self._tf(FRAME_ID, LINK_NAME)
        if pos is None:
            self.done.set()
            return
        c = Constraints()
        c.position_constraints.append(self._pos_constraint(pos.x+dx, pos.y+dy, pos.z+dz))
        c.joint_constraints.extend(self._lock_jc())
        self._send(c)

    def move_xyz_absolute(self, x, y, z):
        if not self.done.is_set():
            self.get_logger().warn("Still executing.")
            return
        self.done.clear()
        c = Constraints()
        c.position_constraints.append(self._pos_constraint(x, y, z))
        c.joint_constraints.extend(self._lock_jc())
        self._send(c)

    def go_home(self):
        if not self.done.is_set():
            self.get_logger().warn("Still executing.")
            return
        self.done.clear()

        c = Constraints()
        for jn in self.arm_joint_names:
            c.joint_constraints.append(self._joint_constraint(jn, self.home_joints[jn], tol=0.03))

        self.get_logger().info(
            "HOME -> " + ", ".join(f"{jn}={math.degrees(self.home_joints[jn]):.0f}°" for jn in self.arm_joint_names)
        )
        self._send(c)

    # --- Rotation: joint-space control of joints 1-3 ---
    def rotate_joint_axis(self, axis: str, angle_rad: float):
        if not self.done.is_set():
            self.get_logger().warn("Still executing.")
            return
        self.done.clear()

        target_joint = ROT_AXIS_TO_JOINT[axis]
        if any(jn not in self.joints for jn in ["Joint_1", "Joint_2", "Joint_3"]):
            self.get_logger().error("Missing joint states for Joint_1/Joint_2/Joint_3")
            self.done.set()
            return

        target = self._clamp_joint(target_joint, self.joints[target_joint] + angle_rad)
        if abs(target - self.joints[target_joint]) < 1e-6:
            self.get_logger().warn(f"{target_joint} already at limit")
            self.done.set()
            return

        c = Constraints()
        for joint_name in ["Joint_1", "Joint_2", "Joint_3"]:
            position = target if joint_name == target_joint else self.joints[joint_name]
            c.joint_constraints.append(self._joint_constraint(joint_name, position))
        c.joint_constraints.extend(self._lock_jc())
        self.get_logger().info(
            f"Target {target_joint}: {math.degrees(self.joints[target_joint]):+.1f}° "
            f"-> {math.degrees(target):+.1f}°"
        )
        self._send(c)

    def _send(self, c: Constraints):
        ok, reason = self.collision_guard.check_current_state(self.joints)
        if not ok:
            reason_l = (reason or "").lower()
            if "timed out" in reason_l or "unavailable" in reason_l:
                self.get_logger().warn(
                    f"Collision guard check skipped ({reason}). Proceeding with command."
                )
            else:
                self.get_logger().warn(f"Blocked by collision guard: {reason}")
                self.done.set()
                return

        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.allowed_planning_time = 3.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.goal_constraints.append(c)
        self._client.wait_for_server()
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, f):
        gh = f.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected.")
            self.done.set()
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, f):
        val = f.result().result.error_code.val
        self.get_logger().info("Done." if val == 1 else f"Failed (code {val})")
        self.done.set()

    def _execute(self, r, source: str = "cli"):
        if r[0] == "quit":
            self.get_logger().info("Exit command ignored from ROS source." if source == "ros" else "Exiting...")
            if source != "ros":
                rclpy.shutdown()
            return
        if r[0] == "print":
            self.print_status()
            return
        if r[0] == "home":
            self.go_home()
            return
        if r[0] == "lock":
            self.lock()
            return
        if r[0] == "unlock":
            self.unlock()
            return
        if r[0] == "xyz":
            _, dx, dy, dz, cmd = r
            cm = (dx**2 + dy**2 + dz**2) ** 0.5 * 100
            self.get_logger().info(
                f"XYZ {cm:.1f}cm world {AXIS_LABEL[cmd]}"
                + ("  [LOCKED]" if self.locked else "")
            )
            self.move_xyz(dx, dy, dz)
            return
        if r[0] == "rot":
            _, axis, angle, desc = r
            self.get_logger().info(
                f"ROT {desc}  {math.degrees(angle):+.1f}°"
                + ("  [LOCKED]" if self.locked else "")
            )
            self.rotate_joint_axis(axis, angle)
            return
        if r[0] == "bad":
            self.get_logger().warn(f"Bad command ({source}): {r[1]}")


def parse(line):
    parts = line.strip().lower().split()
    if not parts:
        return None
    cmd = parts[0]
    if cmd == "x": return ("quit",)
    if cmd == "p": return ("print",)
    if cmd == "h": return ("home",)
    if cmd == "c": return ("lock",)
    if cmd == "u": return ("unlock",)
    if cmd in DIRECTION_MAP:
        try:
            cm = DEFAULT_CM if len(parts) == 1 else float(parts[1])
        except ValueError:
            return ("bad", f"bad distance '{parts[1]}'")
        m = cm / 100.0
        sx, sy, sz = DIRECTION_MAP[cmd]
        return ("xyz", sx*m, sy*m, sz*m, cmd)
    if cmd in ROT_CMDS:
        try:
            deg = DEFAULT_DEG if len(parts) == 1 else float(parts[1])
        except ValueError:
            return ("bad", f"bad angle '{parts[1]}'")
        axis, sign, desc = ROT_CMDS[cmd]
        return ("rot", axis, sign * math.radians(deg), desc)
    return ("bad", f"unknown '{cmd}'")


def run(node: Teleop):
    settings = termios.tcgetattr(sys.stdin)
    print(msg)
    try:
        while rclpy.ok():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            lock_str = (f"LOCKED(J4={math.degrees(node.locked_pitch):.0f}° "
                        f"J5={math.degrees(node.locked_twist):.0f}°)"
                        if node.locked else "free")
            sys.stdout.write(f"[{lock_str}] cmd> ")
            sys.stdout.flush()
            try:
                line = sys.stdin.readline()
            except EOFError:
                break
            if not line:
                continue
            r = parse(line)
            if r is None:
                continue
            if r[0] == "bad":
                print(f"  Error: {r[1]}")
                print("  Try: h | w 5 | q 10 | rz 30 | ry- 15 | c | u | p | x")
                continue
            node._execute(r, source="cli")
            if r[0] == "quit":
                break
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = Teleop()
    if sys.stdin.isatty():
        threading.Thread(target=run, args=(node,), daemon=True).start()
    else:
        node.get_logger().info("No TTY detected: keyboard CLI disabled; ROS topic control only.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()
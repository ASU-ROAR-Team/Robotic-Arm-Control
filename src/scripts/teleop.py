#!/usr/bin/env python3
"""
teleop.py — GUI teleop for 6DOF pose control with gripper control.

Features:
- World-frame XYZ jogging
- Optional fixed-orientation pose solving
- Orientation presets and editable RPY targets
- Single-slider gripper control with open/close shortcuts
- No joint locking; orientation is enforced through full pose IK
"""

import argparse
import math
import os
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from queue import Empty, Queue
import tkinter as tk
from tkinter import scrolledtext

import rclpy
from builtin_interfaces.msg import Duration as BuiltinDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

LINK_NAME = "link_6"
GROUP_NAME = "arm_controller"
FRAME_ID = "world"
BASE_FRAME = "base_link"
SEMANTIC_REFERENCE_FRAME = "link_1"
HAND_CONTROLLER = "/hand_controller_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/joint_states"
DEFAULT_CM = 1.0
POSITION_TOL = 0.01
ORIENTATION_TOL = 0.20
DISPLAY_JOINTS = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
HOME_JOINTS = {
    "joint_0": 0.0,
    "joint_1": 0.0,
    "joint_2": 0.0,
    "joint_3": 0.0,
    "joint_4": 0.0,
    "joint_5": 0.0,
}
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.069
GRIPPER_OPEN_BUTTON = 0.0
GRIPPER_CLOSE_BUTTON = 0.069
GRIPPER_TRAJECTORY_SECONDS = 0.35
ORIENTATION_PRESETS_DEG = {
    "Look Forward": (180.0, 0.0, 0.0),
    "Look Down": (173.0, 0.3, -90.0),
    "Look Up": (7.0, -0.3, -90.0),
    "Look Right": (179.0, -2.0, -90.0),
    "Look Left": (179.0, 2.0, 90.0),
}
TOOL_AXIS_VECTORS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}
SEMANTIC_AXIS_TO_LINK1_X = {
    "Forward": "+Z",
    "Down": "+Y",
    "Up": "-Y",
    "Right": "+X",
    "Left": "-X",
    "Backward": "-Z",
}
WRIST_JOINT_LIMIT = 1.6581
WRIST_SEMANTIC_JOINTS = ("joint_3", "joint_4", "joint_5")
SEMANTIC_CHAIN_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5")
SEMANTIC_PREFIX_LINK = "link_3"
SEMANTIC_DEFAULT_VERTICAL_MODE = "level"
SEMANTIC_DEFAULT_HORIZONTAL_MODE = "center"
SEMANTIC_UPSTREAM_JOINTS = ("joint_0", "joint_1", "joint_2")
SEMANTIC_STATE_JOINTS = ("joint_3", "joint_4", "joint_5")
SEMANTIC_SCORE_WEIGHTS = {
    "changed_primary": 12000.0,
    "changed_secondary": 3200.0,
    "state_hold": 450.0,
    "upright": 160.0,
    "memory": 220.0,
    "wrist_motion": 40.0,
    "upstream_motion": 60.0,
}
SEMANTIC_FALLBACK_ALIGNMENT_FLOOR = 0.84
SEMANTIC_FALLBACK_SECONDARY_FLOOR = 0.58
SEMANTIC_RELAXED_TOLERANCE_SCALE = 1.5
SEMANTIC_MAX_GOAL_TOLERANCE = 0.30
SEMANTIC_MAX_PLANNER_ATTEMPTS = 4
SEMANTIC_VALIDATION_DELAY_SEC = 0.35
SEMANTIC_FK_TF_WARN_DEG = 6.0
SEMANTIC_MEMORY_NEARBY_JOINT_DEG = 8.0
SEMANTIC_VERTICAL_MODE_LABELS = {
    "down": "Down",
    "level": "Level",
    "up": "Up",
}
SEMANTIC_HORIZONTAL_MODE_LABELS = {
    "left": "Left",
    "center": "Center",
    "right": "Right",
}
SEMANTIC_COMMAND_TARGETS = {
    "Forward": {"vertical": "level", "horizontal": "center", "requested_stages": ("vertical", "horizontal")},
    "Down": {"vertical": "down", "horizontal": None, "requested_stages": ("vertical",)},
    "Up": {"vertical": "up", "horizontal": None, "requested_stages": ("vertical",)},
    "Right": {"vertical": None, "horizontal": "right", "requested_stages": ("horizontal",)},
    "Left": {"vertical": None, "horizontal": "left", "requested_stages": ("horizontal",)},
}
SEMANTIC_VERTICAL_PROFILES = {
    "down": {
        "joint_4_samples_deg": [-35.0, -20.0, -10.0, 0.0, 10.0, 20.0, 35.0],
        "primary_axis": "+Y",
        "primary_target": (1.0, 0.0, 0.0),
        "secondary_axis": "+X",
        "secondary_target": (0.0, 1.0, 0.0),
        "primary_min": 0.93,
        "secondary_min": 0.72,
        "goal_tolerance": 0.18,
        "memory_bias_joint": "joint_4",
    },
    "level": {
        "joint_4_samples_deg": [20.0, 35.0, 50.0, 65.0, 80.0, 90.0, 100.0],
        "primary_axis": "+Z",
        "primary_target": (1.0, 0.0, 0.0),
        "secondary_axis": "+Y",
        "secondary_target": (0.0, 0.0, 1.0),
        "primary_min": 0.95,
        "secondary_min": 0.82,
        "goal_tolerance": 0.16,
        "memory_bias_joint": "joint_4",
    },
    "up": {
        "joint_4_samples_deg": [-95.0, -80.0, -65.0, -50.0, -35.0, -20.0, -5.0],
        "primary_axis": "-Y",
        "primary_target": (1.0, 0.0, 0.0),
        "secondary_axis": "+X",
        "secondary_target": (0.0, 1.0, 0.0),
        "primary_min": 0.90,
        "secondary_min": 0.68,
        "goal_tolerance": 0.20,
        "memory_bias_joint": "joint_4",
    },
}
SEMANTIC_HORIZONTAL_PROFILES = {
    "left": {
        "joint_3_samples_deg": [-95.0, -80.0, -65.0, -50.0, -35.0, -20.0, -5.0],
        "primary_axis": "-X",
        "primary_target": (1.0, 0.0, 0.0),
        "secondary_axis": "+Y",
        "secondary_target": (0.0, 0.0, 1.0),
        "primary_min": 0.90,
        "secondary_min": 0.66,
        "goal_tolerance": 0.20,
        "memory_bias_joint": "joint_3",
    },
    "center": {
        "joint_3_samples_deg": [-25.0, -15.0, -5.0, 0.0, 5.0, 15.0, 25.0],
        "primary_axis": "+X",
        "primary_target": (0.0, 1.0, 0.0),
        "secondary_axis": "+Y",
        "secondary_target": (0.0, 0.0, 1.0),
        "primary_min": 0.74,
        "secondary_min": 0.62,
        "goal_tolerance": 0.16,
        "memory_bias_joint": "joint_3",
    },
    "right": {
        "joint_3_samples_deg": [5.0, 20.0, 35.0, 50.0, 65.0, 80.0, 95.0],
        "primary_axis": "+X",
        "primary_target": (1.0, 0.0, 0.0),
        "secondary_axis": "+Y",
        "secondary_target": (0.0, 0.0, 1.0),
        "primary_min": 0.90,
        "secondary_min": 0.66,
        "goal_tolerance": 0.20,
        "memory_bias_joint": "joint_3",
    },
}
SEMANTIC_JOINT_5_SAMPLES_DEG = [-90.0, -60.0, -35.0, -15.0, 0.0, 15.0, 35.0, 60.0, 90.0]
SEMANTIC_JOINT_5_SETTLE_DEG = 12.0
SEMANTIC_STAGE_HOLD_NEARBY_DEG = 10.0
SEMANTIC_STATE_MATCH_DEG = 7.0
SEMANTIC_UPSTREAM_TOLERANCES = {
    "tight": {"joint_0": 0.025, "joint_1": 0.025, "joint_2": 0.03},
    "relaxed": {"joint_0": 0.06, "joint_1": 0.04, "joint_2": 0.05},
    "helper": {"joint_0": 0.16, "joint_1": 0.05, "joint_2": 0.06},
}
SEMANTIC_JOINT_LIMIT_FALLBACKS = {
    "joint_1": (-2.635, 0.017),
    "joint_2": (-0.017, 3.159),
    "joint_3": (-WRIST_JOINT_LIMIT, WRIST_JOINT_LIMIT),
    "joint_4": (-WRIST_JOINT_LIMIT, WRIST_JOINT_LIMIT),
    "joint_5": (-WRIST_JOINT_LIMIT, WRIST_JOINT_LIMIT),
}
SEMANTIC_URDF_CANDIDATES = (
    Path(__file__).resolve().parents[1] / "sixdof_pkg" / "urdf" / "roar_variant.urdf.xacro",
    Path(__file__).resolve().parents[1] / "sixdof_moveit" / "config" / "sixdof_pkg.urdf.xacro",
)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_from_axis_angle(axis: tuple[float, float, float], angle: float) -> tuple[float, float, float, float]:
    axis = normalize_vector(axis)
    half_angle = angle * 0.5
    sin_half = math.sin(half_angle)
    return (
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half,
        math.cos(half_angle),
    )


def euler_from_quat(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def joint_constraint(joint_name: str, position: float, tolerance: float) -> JointConstraint:
    constraint = JointConstraint()
    constraint.joint_name = joint_name
    constraint.position = position
    constraint.tolerance_above = tolerance
    constraint.tolerance_below = tolerance
    constraint.weight = 1.0
    return constraint


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude < 1e-9:
        return (0.0, 0.0, 0.0)
    return (x / magnitude, y / magnitude, z / magnitude)


def parse_xyz_attribute(text: str | None, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if not text:
        return default
    values = [float(part) for part in text.split()]
    if len(values) != 3:
        return default
    return (values[0], values[1], values[2])


def closest_semantic_direction(vector: tuple[float, float, float]) -> tuple[str | None, float, tuple[float, float, float]]:
    unit = normalize_vector(vector)
    best_name = None
    best_score = -1.0
    for name, axis_name in SEMANTIC_AXIS_TO_LINK1_X.items():
        axis = TOOL_AXIS_VECTORS[axis_name]
        score = unit[0] * axis[0] + unit[1] * axis[1] + unit[2] * axis[2]
        if score > best_score:
            best_name = name
            best_score = score
    return best_name, best_score, unit


def semantic_name_for_axis(axis_name: str) -> str | None:
    for semantic, mapped_axis in SEMANTIC_AXIS_TO_LINK1_X.items():
        if mapped_axis == axis_name:
            return semantic
    return None


def clamp_value(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Teleop(Node):
    def __init__(self, log_callback=None):
        super().__init__("sixdof_pose_teleop")
        self._client = ActionClient(self, MoveGroup, "move_action")
        self._hand_client = ActionClient(self, FollowJointTrajectory, HAND_CONTROLLER)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.done = threading.Event()
        self.done.set()
        self.hand_done = threading.Event()
        self.hand_done.set()
        self.joints: dict[str, float] = {}
        self.log_callback = log_callback
        self.maintain_orientation = True
        self.target_orientation = quat_from_euler(0.0, 0.0, 0.0)
        self.target_orientation_rpy_deg = [0.0, 0.0, 0.0]
        self.last_semantic_strategy = "none"
        self.last_semantic_validation = "none"
        self.last_semantic_skip = "none"
        self.semantic_state = {
            "vertical_mode": None,
            "horizontal_mode": None,
            "last_successful_state": None,
            "last_successful_candidate": None,
            "last_successful_validation": "none",
            "last_requested_command": "none",
            "seed_source": "uninitialized",
        }
        self._semantic_validation_timer = None
        self._active_goal_context = None
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._js, 10)
        # Reference frame (can be changed at runtime)
        self.reference_frame = FRAME_ID
        self.create_subscription(String, '/reference_frame', self._on_ref_frame, 10)
        self._ref_capture_timer = None
        # Publisher to request reference frame pose changes
        self._ref_pose_pub = self.create_publisher(PoseStamped, '/ee_reference_pose', 10)
        self._semantic_model = self._load_semantic_kinematic_model()
        if self._semantic_model is not None:
            self._log(
                "info",
                f"Semantic kinematic model loaded from {self._semantic_model['urdf_path']} "
                f"using prefix {SEMANTIC_REFERENCE_FRAME}->{self._semantic_model['prefix_link']}",
            )

    def _log(self, level, text):
        logger = self.get_logger()
        if level == "error":
            logger.error(text)
        elif level == "warn":
            logger.warning(text)
        else:
            logger.info(text)
        if self.log_callback is not None:
            self.log_callback(level, text)

    def rotate_reference(self, axis: str, degrees: float):
        # Rotate the ee_ref frame about a world axis ('X','Y','Z') by degrees, keeping position
        # Find current ee_ref pose in world (fallback to EE link)
        transform = self._tf_transform('world', 'ee_ref')
        if transform is None:
            transform = self._tf_transform('world', LINK_NAME)
            if transform is None:
                self._log('error', 'No ee_ref or EE transform available to rotate')
                return

        tx = transform.translation.x
        ty = transform.translation.y
        tz = transform.translation.z
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w

        # rotation quaternion about specified world axis
        a = axis.upper()
        rad = math.radians(degrees)
        if a == 'X':
            qrot = quat_from_euler(rad, 0.0, 0.0)
        elif a == 'Y':
            qrot = quat_from_euler(0.0, rad, 0.0)
        else:
            qrot = quat_from_euler(0.0, 0.0, rad)

        # q_new = qrot * q_current (apply rotation in world frame)
        def quat_mult(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
            return (qx, qy, qz, qw)

        q_current = (qx, qy, qz, qw)
        q_new = quat_mult(qrot, q_current)

        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tx
        ps.pose.position.y = ty
        ps.pose.position.z = tz
        ps.pose.orientation.x = q_new[0]
        ps.pose.orientation.y = q_new[1]
        ps.pose.orientation.z = q_new[2]
        ps.pose.orientation.w = q_new[3]
        self._ref_pose_pub.publish(ps)
        self._log('info', f'Rotated ee_ref around world {a} by {degrees:.1f} deg')
        # done

    def _on_ref_frame(self, msg: String):
        try:
            self.set_reference_frame(msg.data)
        except Exception:
            pass

    def _js(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    def _tf_transform(self, parent, child):
        # Try direct lookup first
        try:
            if self.tf_buffer.can_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
                return self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                ).transform
        except Exception:
            pass

        # Fallback: attempt to compute parent->child via world frame if possible
        try:
            if parent == 'world' or child == 'world':
                return None
            # need T_world_parent and T_world_child
            if not self.tf_buffer.can_transform('world', parent, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
                return None
            if not self.tf_buffer.can_transform('world', child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
                return None
            t_world_parent = self.tf_buffer.lookup_transform('world', parent, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            t_world_child = self.tf_buffer.lookup_transform('world', child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            # compute T_parent_child = inv(T_world_parent) * T_world_child
            inv_r, inv_t = self._invert_transform(t_world_parent.transform)
            comp = self._compose_transform(inv_r, inv_t, t_world_child.transform)
            # return a Transform-like object with translation and rotation
            class SimpleTransform:
                pass
            st = SimpleTransform()
            st.translation = type('T', (), {})()
            st.rotation = type('Q', (), {})()
            st.translation.x, st.translation.y, st.translation.z = comp[1]
            st.rotation.x, st.rotation.y, st.rotation.z, st.rotation.w = comp[0]
            self._log('info', f'Used world-based TF fallback for {parent}->{child}')
            return st
        except Exception as exc:
            self._log('warn', f'Fallback TF {parent}->{child} failed: {exc}')
            return None

    def _invert_transform(self, transform):
        # transform: geometry_msgs/Transform
        q = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        t = (transform.translation.x, transform.translation.y, transform.translation.z)
        # inverse rotation is conjugate
        qx, qy, qz, qw = q
        inv_q = (-qx, -qy, -qz, qw)
        # rotate -t by inv_q
        rt = self._rotate_vector(inv_q, (-t[0], -t[1], -t[2]))
        return inv_q, rt

    def _compose_transform(self, q1, t1, transform2):
        # q1: (x,y,z,w) rotation, t1: (x,y,z) translation; transform2 has rotation & translation
        q2 = (transform2.rotation.x, transform2.rotation.y, transform2.rotation.z, transform2.rotation.w)
        t2 = (transform2.translation.x, transform2.translation.y, transform2.translation.z)
        # composed rotation q = q1 * q2
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        # rotated translation: t = t1 + rotate(q1, t2)
        rt2 = self._rotate_vector(q1, t2)
        tx = t1[0] + rt2[0]
        ty = t1[1] + rt2[1]
        tz = t1[2] + rt2[2]
        return (qx, qy, qz, qw), (tx, ty, tz)

    def _rotate_vector(self, q, v):
        # rotate vector v by quaternion q
        x, y, z, w = q
        vx, vy, vz = v
        # q * v * q_conj
        # compute q*v
        ix =  w * vx + y * vz - z * vy
        iy =  w * vy + z * vx - x * vz
        iz =  w * vz + x * vy - y * vx
        iw = -x * vx - y * vy - z * vz
        # result = (qv) * q_conj
        rx = ix * w + iw * -x + iy * -z - iz * -y
        ry = iy * w + iw * -y + iz * -x - ix * -z
        rz = iz * w + iw * -z + ix * -y - iy * -x
        return (rx, ry, rz)

    def _quat_mult(self, q1, q2):
        # multiply q1 * q2, quaternions as (x,y,z,w)
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        qw = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        qx = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        qy = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        qz = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return (qx, qy, qz, qw)

    def _quat_conjugate(self, q):
        x, y, z, w = q
        return (-x, -y, -z, w)

    def _normalize_quat(self, q):
        x, y, z, w = q
        magnitude = math.sqrt(x * x + y * y + z * z + w * w)
        if magnitude < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / magnitude, y / magnitude, z / magnitude, w / magnitude)

    def _quat_from_two_vectors(self, v_from, v_to):
        # return quaternion rotating v_from -> v_to (shortest arc), as (x,y,z,w)
        fx, fy, fz = v_from
        tx, ty, tz = v_to
        # normalize
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        tmag = math.sqrt(tx * tx + ty * ty + tz * tz)
        if fmag < 1e-9 or tmag < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        v1 = (fx / fmag, fy / fmag, fz / fmag)
        v2 = (tx / tmag, ty / tmag, tz / tmag)
        dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
        if dot >= 1.0 - 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        if dot <= -1.0 + 1e-12:
            # 180 degree rotation: pick an orthogonal axis
            # find axis orthogonal to v_from
            if abs(v1[0]) < abs(v1[1]):
                ort = (1.0, 0.0, 0.0)
            else:
                ort = (0.0, 1.0, 0.0)
            # cross product
            cx = v1[1] * ort[2] - v1[2] * ort[1]
            cy = v1[2] * ort[0] - v1[0] * ort[2]
            cz = v1[0] * ort[1] - v1[1] * ort[0]
            mag = math.sqrt(cx * cx + cy * cy + cz * cz)
            if mag < 1e-9:
                return (0.0, 0.0, 0.0, 1.0)
            sx, sy, sz = cx / mag, cy / mag, cz / mag
            return (sx, sy, sz, 0.0)
        # standard case
        cx = v1[1] * v2[2] - v1[2] * v2[1]
        cy = v1[2] * v2[0] - v1[0] * v2[2]
        cz = v1[0] * v2[1] - v1[1] * v2[0]
        s = math.sqrt((1.0 + dot) * 2.0)
        invs = 1.0 / s
        qx = cx * invs
        qy = cy * invs
        qz = cz * invs
        qw = 0.5 * s
        return (qx, qy, qz, qw)

    def _load_semantic_kinematic_model(self):
        last_error = None
        for urdf_path in SEMANTIC_URDF_CANDIDATES:
            if not urdf_path.exists():
                continue
            try:
                if urdf_path.suffix == ".xacro":
                    xml_text = subprocess.check_output(
                        ["xacro", str(urdf_path)],
                        text=True,
                        env=os.environ.copy(),
                    )
                    root = ET.fromstring(xml_text)
                else:
                    root = ET.parse(urdf_path).getroot()
                joints: dict[str, dict[str, object]] = {}
                for joint_element in root.findall("joint"):
                    joint_name = joint_element.attrib.get("name")
                    if joint_name is None:
                        continue
                    parent_element = joint_element.find("parent")
                    child_element = joint_element.find("child")
                    origin_element = joint_element.find("origin")
                    axis_element = joint_element.find("axis")
                    limit_element = joint_element.find("limit")
                    origin_xyz = parse_xyz_attribute(
                        origin_element.attrib.get("xyz") if origin_element is not None else None,
                        (0.0, 0.0, 0.0),
                    )
                    origin_rpy = parse_xyz_attribute(
                        origin_element.attrib.get("rpy") if origin_element is not None else None,
                        (0.0, 0.0, 0.0),
                    )
                    lower = None
                    upper = None
                    if limit_element is not None:
                        lower_text = limit_element.attrib.get("lower")
                        upper_text = limit_element.attrib.get("upper")
                        lower = float(lower_text) if lower_text is not None else None
                        upper = float(upper_text) if upper_text is not None else None
                    joints[joint_name] = {
                        "type": joint_element.attrib.get("type", "fixed"),
                        "parent": parent_element.attrib.get("link") if parent_element is not None else None,
                        "child": child_element.attrib.get("link") if child_element is not None else None,
                        "origin_xyz": origin_xyz,
                        "origin_rpy": origin_rpy,
                        "origin_quat": quat_from_euler(*origin_rpy),
                        "axis": normalize_vector(
                            parse_xyz_attribute(
                                axis_element.attrib.get("xyz") if axis_element is not None else None,
                                (1.0, 0.0, 0.0),
                            )
                        ),
                        "limits": (lower, upper),
                    }
                missing = [joint_name for joint_name in SEMANTIC_CHAIN_JOINTS if joint_name not in joints]
                if missing:
                    raise ValueError("missing joints: " + ", ".join(missing))
                return {
                    "urdf_path": str(urdf_path),
                    "joints": joints,
                    "prefix_link": joints["joint_3"]["parent"],
                }
            except Exception as exc:
                last_error = f"{urdf_path}: {exc}"
        if last_error is not None:
            self._log("warn", f"Semantic URDF model unavailable: {last_error}")
        else:
            self._log("warn", "Semantic URDF model unavailable: no candidate URDF path found")
        return None

    def _semantic_joint_limit(self, joint_name: str) -> tuple[float, float]:
        if self._semantic_model is not None:
            joint_limits = self._semantic_model["joints"][joint_name]["limits"]
            lower = joint_limits[0]
            upper = joint_limits[1]
            if lower is not None and upper is not None:
                return (float(lower), float(upper))
        return SEMANTIC_JOINT_LIMIT_FALLBACKS.get(joint_name, (-WRIST_JOINT_LIMIT, WRIST_JOINT_LIMIT))

    def _semantic_joint_state_snapshot(self, joint_names: tuple[str, ...]) -> tuple[dict[str, float], list[str]]:
        snapshot: dict[str, float] = {}
        missing: list[str] = []
        for joint_name in joint_names:
            low, high = self._semantic_joint_limit(joint_name)
            if joint_name not in self.joints:
                missing.append(joint_name)
            snapshot[joint_name] = clamp_value(self.joints.get(joint_name, 0.0), low, high)
        return snapshot, missing

    def _semantic_transform_quaternion(self, transform) -> tuple[float, float, float, float]:
        return self._normalize_quat(
            (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        )

    def _semantic_chain_quaternion(
        self,
        joint_values: dict[str, float],
        joint_names: tuple[str, ...],
        initial_q: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    ) -> tuple[float, float, float, float] | None:
        if self._semantic_model is None:
            return None
        q = self._normalize_quat(initial_q)
        for joint_name in joint_names:
            joint_model = self._semantic_model["joints"][joint_name]
            # URDF joint convention: parent->child orientation is the fixed
            # joint-origin rotation followed by rotation about the joint axis
            # expressed in that joint frame.
            q = self._quat_mult(q, joint_model["origin_quat"])
            if joint_model["type"] != "fixed":
                q = self._quat_mult(q, quat_from_axis_angle(joint_model["axis"], joint_values.get(joint_name, 0.0)))
        return self._normalize_quat(q)

    def _semantic_quat_angle_deg(self, q1, q2) -> float:
        relative = self._normalize_quat(self._quat_mult(self._quat_conjugate(q1), q2))
        return math.degrees(2.0 * math.acos(clamp_value(abs(relative[3]), -1.0, 1.0)))

    def _semantic_prediction_context(self):
        if self._semantic_model is None:
            return None

        wrist_state, missing_wrist = self._semantic_joint_state_snapshot(WRIST_SEMANTIC_JOINTS)
        full_chain_state, missing_chain = self._semantic_joint_state_snapshot(SEMANTIC_CHAIN_JOINTS)
        prefix_link = self._semantic_model["prefix_link"]
        prefix_transform = self._tf_transform(SEMANTIC_REFERENCE_FRAME, prefix_link)
        if prefix_transform is not None:
            # This keeps the upstream chain grounded in the live robot/TF state
            # and only models the semantic wrist subchain locally.
            prefix_q = self._semantic_transform_quaternion(prefix_transform)
            current_model_q = self._semantic_chain_quaternion(wrist_state, WRIST_SEMANTIC_JOINTS, initial_q=prefix_q)
            return {
                "mode": "tf_prefix_subchain_fk",
                "source": (
                    f"actual TF {SEMANTIC_REFERENCE_FRAME}->{prefix_link} "
                    f"+ URDF FK over {', '.join(WRIST_SEMANTIC_JOINTS)}"
                ),
                "prefix_link": prefix_link,
                "prefix_q": prefix_q,
                "current_model_q": current_model_q,
                "current_wrist_state": wrist_state,
                "full_chain_state": full_chain_state,
                "missing_joint_states": missing_wrist,
            }

        current_model_q = self._semantic_chain_quaternion(full_chain_state, SEMANTIC_CHAIN_JOINTS)
        return {
            "mode": "joint_state_full_fk",
            "source": f"joint-state URDF FK over {', '.join(SEMANTIC_CHAIN_JOINTS)}",
            "prefix_link": None,
            "prefix_q": None,
            "current_model_q": current_model_q,
            "current_wrist_state": wrist_state,
            "full_chain_state": full_chain_state,
            "missing_joint_states": missing_chain,
        }

    def _semantic_predict_candidate_quaternion(self, candidate: dict[str, float], context):
        if context["mode"] == "tf_prefix_subchain_fk":
            return self._semantic_chain_quaternion(candidate, WRIST_SEMANTIC_JOINTS, initial_q=context["prefix_q"])
        joint_values = dict(context["full_chain_state"])
        joint_values.update(candidate)
        return self._semantic_chain_quaternion(joint_values, SEMANTIC_CHAIN_JOINTS)

    def _semantic_sample_summary(self, samples: dict[str, list[float]]) -> str:
        parts = []
        for joint_name in WRIST_SEMANTIC_JOINTS:
            degrees = ", ".join(f"{math.degrees(value):+.1f}" for value in samples[joint_name])
            parts.append(f"{joint_name}_deg=[{degrees}]")
        return "; ".join(parts)

    def _semantic_alignment_from_quaternion(self, q):
        q = self._normalize_quat(q)
        axis_vectors = {
            axis_name: normalize_vector(self._rotate_vector(q, axis_vector))
            for axis_name, axis_vector in TOOL_AXIS_VECTORS.items()
        }

        best_axis_name = None
        best_axis_vector = (0.0, 0.0, 0.0)
        best_score = -1.0
        for axis_name in ["+Z", "+Y", "-Y", "+X", "-X", "-Z"]:
            score = axis_vectors[axis_name][0]
            if score > best_score:
                best_axis_name = axis_name
                best_axis_vector = axis_vectors[axis_name]
                best_score = score

        semantic = semantic_name_for_axis(best_axis_name) if best_axis_name is not None else None

        # Assume the camera's visible top/slot side corresponds to tool +Y.
        # Positive link_1 +Z means upright, negative means upside-down.
        upright_score = axis_vectors["+Y"][2]
        if upright_score > 0.25:
            upright_state = "upright"
        elif upright_score < -0.25:
            upright_state = "upside-down"
        else:
            upright_state = "sideways"

        return {
            "semantic": semantic,
            "axis_name": best_axis_name,
            "axis_vector": best_axis_vector,
            "score": best_score,
            "upright_state": upright_state,
            "upright_score": upright_score,
            "axis_vectors": axis_vectors,
        }

    def _semantic_alignment_state(self):
        transform = self._tf_transform(SEMANTIC_REFERENCE_FRAME, LINK_NAME)
        if transform is None:
            return None
        q = self._semantic_transform_quaternion(transform)
        return self._semantic_alignment_from_quaternion(q)

    def _semantic_mode_label(self, stage: str, mode: str | None) -> str:
        if mode is None:
            return "unknown"
        labels = SEMANTIC_VERTICAL_MODE_LABELS if stage == "vertical" else SEMANTIC_HORIZONTAL_MODE_LABELS
        return labels.get(mode, mode)

    def _semantic_stage_description(self, target_state: dict) -> str:
        changed = target_state["changed_stages"]
        if not changed:
            return "none"
        if len(changed) == 2:
            return "stage1+stage2"
        return "stage1" if changed[0] == "vertical" else "stage2"

    def _semantic_axis_score(self, axis_vectors: dict[str, tuple[float, float, float]], axis_name: str, target_vector) -> float:
        vector = axis_vectors[axis_name]
        target = normalize_vector(target_vector)
        return vector[0] * target[0] + vector[1] * target[1] + vector[2] * target[2]

    def _semantic_stage_metrics(self, alignment: dict, stage: str, mode: str) -> dict:
        profiles = SEMANTIC_VERTICAL_PROFILES if stage == "vertical" else SEMANTIC_HORIZONTAL_PROFILES
        profile = profiles[mode]
        axis_vectors = alignment["axis_vectors"]
        primary = self._semantic_axis_score(axis_vectors, profile["primary_axis"], profile["primary_target"])
        secondary = self._semantic_axis_score(axis_vectors, profile["secondary_axis"], profile["secondary_target"])
        return {
            "mode": mode,
            "primary": primary,
            "secondary": secondary,
            "primary_min": profile["primary_min"],
            "secondary_min": profile["secondary_min"],
            "primary_axis": profile["primary_axis"],
            "secondary_axis": profile["secondary_axis"],
        }

    def _semantic_observed_modes(self, alignment: dict) -> dict:
        vertical_mode = max(
            SEMANTIC_VERTICAL_PROFILES,
            key=lambda mode: (
                self._semantic_stage_metrics(alignment, "vertical", mode)["primary"],
                self._semantic_stage_metrics(alignment, "vertical", mode)["secondary"],
            ),
        )
        horizontal_mode = max(
            SEMANTIC_HORIZONTAL_PROFILES,
            key=lambda mode: (
                self._semantic_stage_metrics(alignment, "horizontal", mode)["primary"],
                self._semantic_stage_metrics(alignment, "horizontal", mode)["secondary"],
            ),
        )
        return {"vertical": vertical_mode, "horizontal": horizontal_mode}

    def _semantic_seed_state_if_needed(self, alignment: dict | None = None):
        if self.semantic_state["vertical_mode"] is not None and self.semantic_state["horizontal_mode"] is not None:
            return
        if alignment is None:
            alignment = self._semantic_alignment_state()
        if alignment is None:
            self.semantic_state["vertical_mode"] = SEMANTIC_DEFAULT_VERTICAL_MODE
            self.semantic_state["horizontal_mode"] = SEMANTIC_DEFAULT_HORIZONTAL_MODE
            self.semantic_state["seed_source"] = "default semantic state"
            return
        observed = self._semantic_observed_modes(alignment)
        self.semantic_state["vertical_mode"] = observed["vertical"]
        self.semantic_state["horizontal_mode"] = observed["horizontal"]
        self.semantic_state["seed_source"] = "observed TF semantic state"

    def _semantic_target_state(self, command: str) -> dict | None:
        target = SEMANTIC_COMMAND_TARGETS.get(command)
        if target is None:
            return None
        self._semantic_seed_state_if_needed()
        vertical_mode = target["vertical"] or self.semantic_state["vertical_mode"] or SEMANTIC_DEFAULT_VERTICAL_MODE
        horizontal_mode = target["horizontal"] or self.semantic_state["horizontal_mode"] or SEMANTIC_DEFAULT_HORIZONTAL_MODE
        changed_stages = []
        if vertical_mode != self.semantic_state["vertical_mode"]:
            changed_stages.append("vertical")
        if horizontal_mode != self.semantic_state["horizontal_mode"]:
            changed_stages.append("horizontal")
        return {
            "command": command,
            "vertical": vertical_mode,
            "horizontal": horizontal_mode,
            "requested_stages": tuple(target["requested_stages"]),
            "changed_stages": tuple(changed_stages),
        }

    def _semantic_memory_seed(self) -> tuple[dict[str, float], dict[str, float], str]:
        current, _ = self._semantic_joint_state_snapshot(WRIST_SEMANTIC_JOINTS)
        last_success = self.semantic_state["last_successful_candidate"]
        if last_success is None:
            return dict(current), current, "current wrist state"
        seed = {joint_name: last_success.get(joint_name, current[joint_name]) for joint_name in WRIST_SEMANTIC_JOINTS}
        return seed, current, "last successful semantic solution"

    def _semantic_collect_samples(
        self,
        joint_name: str,
        anchor_values_deg: list[float],
        current_value: float,
        seed_value: float,
        expanded: bool,
    ) -> list[float]:
        unique_values: dict[float, float] = {}
        low, high = self._semantic_joint_limit(joint_name)
        nearby_deg = SEMANTIC_STAGE_HOLD_NEARBY_DEG if expanded else (SEMANTIC_STAGE_HOLD_NEARBY_DEG * 0.5)
        sample_values = list(anchor_values_deg)
        sample_values.extend(
            [
                math.degrees(seed_value),
                math.degrees(current_value),
                math.degrees(seed_value + math.radians(nearby_deg)),
                math.degrees(seed_value - math.radians(nearby_deg)),
                math.degrees(current_value + math.radians(nearby_deg)),
                math.degrees(current_value - math.radians(nearby_deg)),
            ]
        )
        for value_deg in sample_values:
            value = clamp_value(math.radians(value_deg), low, high)
            unique_values[round(value, 6)] = value
        return [unique_values[key] for key in sorted(unique_values)]

    def _semantic_candidate_samples(self, target_state: dict, seed_solution: dict[str, float], current: dict[str, float]):
        requested = set(target_state["requested_stages"])
        samples = {
            "joint_3": self._semantic_collect_samples(
                "joint_3",
                SEMANTIC_HORIZONTAL_PROFILES[target_state["horizontal"]]["joint_3_samples_deg"],
                current["joint_3"],
                seed_solution["joint_3"],
                "horizontal" in requested,
            ),
            "joint_4": self._semantic_collect_samples(
                "joint_4",
                SEMANTIC_VERTICAL_PROFILES[target_state["vertical"]]["joint_4_samples_deg"],
                current["joint_4"],
                seed_solution["joint_4"],
                "vertical" in requested,
            ),
            "joint_5": self._semantic_collect_samples(
                "joint_5",
                SEMANTIC_JOINT_5_SAMPLES_DEG,
                current["joint_5"],
                seed_solution["joint_5"],
                True,
            ),
        }
        return samples

    def _semantic_memory_closeness(self, candidate: dict[str, float], seed_solution: dict[str, float]) -> float:
        scores = []
        for joint_name in WRIST_SEMANTIC_JOINTS:
            low, high = self._semantic_joint_limit(joint_name)
            span = max(abs(low), abs(high), 1e-6)
            scores.append(clamp_value(1.0 - abs(candidate[joint_name] - seed_solution[joint_name]) / span, 0.0, 1.0))
        return sum(scores) / len(scores)

    def _semantic_joint_closeness(self, joint_name: str, candidate_value: float, seed_value: float) -> float:
        low, high = self._semantic_joint_limit(joint_name)
        span = max(abs(low), abs(high), 1e-6)
        return clamp_value(1.0 - abs(candidate_value - seed_value) / span, 0.0, 1.0)

    def _semantic_state_hold_score(self, candidate: dict[str, float], target_state: dict, seed_solution: dict[str, float]) -> float:
        hold_scores = []
        if "horizontal" not in target_state["requested_stages"]:
            hold_scores.append(self._semantic_joint_closeness("joint_3", candidate["joint_3"], seed_solution["joint_3"]))
        if "vertical" not in target_state["requested_stages"]:
            hold_scores.append(self._semantic_joint_closeness("joint_4", candidate["joint_4"], seed_solution["joint_4"]))
        hold_scores.append(
            clamp_value(
                1.0 - abs(candidate["joint_5"] - seed_solution["joint_5"]) / max(WRIST_JOINT_LIMIT, 1e-6),
                0.0,
                1.0,
            )
        )
        return sum(hold_scores) / len(hold_scores)

    def _semantic_wrist_motion_penalty(self, candidate: dict[str, float], current: dict[str, float]) -> float:
        motion = 0.0
        for joint_name in WRIST_SEMANTIC_JOINTS:
            low, high = self._semantic_joint_limit(joint_name)
            span = max(abs(low), abs(high), 1e-6)
            motion += abs(candidate[joint_name] - current[joint_name]) / span
        return motion / len(WRIST_SEMANTIC_JOINTS)

    def _semantic_validation_metrics(self, target_state: dict, alignment: dict) -> dict:
        vertical_metrics = self._semantic_stage_metrics(alignment, "vertical", target_state["vertical"])
        horizontal_metrics = self._semantic_stage_metrics(alignment, "horizontal", target_state["horizontal"])
        requested = set(target_state["requested_stages"])
        active_metrics = []
        if "vertical" in requested:
            active_metrics.append(vertical_metrics)
        if "horizontal" in requested:
            active_metrics.append(horizontal_metrics)
        if not active_metrics:
            active_metrics = [vertical_metrics, horizontal_metrics]
        primary_score = sum(metric["primary"] for metric in active_metrics) / len(active_metrics)
        secondary_score = sum(metric["secondary"] for metric in active_metrics) / len(active_metrics)
        return {
            "vertical": vertical_metrics,
            "horizontal": horizontal_metrics,
            "primary_score": primary_score,
            "secondary_score": secondary_score,
            "passed": all(
                metric["primary"] >= metric["primary_min"] and metric["secondary"] >= metric["secondary_min"]
                for metric in active_metrics
            ),
        }

    def _semantic_already_satisfied(self, target_state: dict, alignment: dict | None) -> dict | None:
        if alignment is None:
            return None
        metrics = self._semantic_validation_metrics(target_state, alignment)
        if not metrics["passed"]:
            return None
        seed_solution, current, _ = self._semantic_memory_seed()
        if self._semantic_memory_closeness(current, seed_solution) < 0.70 and self.semantic_state["last_successful_candidate"] is not None:
            return None
        return metrics

    def _semantic_candidate_result(
        self,
        candidate: dict[str, float],
        alignment: dict,
        target_state: dict,
        current: dict[str, float],
        seed_solution: dict[str, float],
    ) -> dict:
        validation = self._semantic_validation_metrics(target_state, alignment)
        requested = set(target_state["requested_stages"])
        changed_metrics = []
        if "vertical" in requested:
            changed_metrics.append(validation["vertical"])
        if "horizontal" in requested:
            changed_metrics.append(validation["horizontal"])
        changed_primary_score = sum(metric["primary"] for metric in changed_metrics) / len(changed_metrics)
        changed_secondary_score = sum(metric["secondary"] for metric in changed_metrics) / len(changed_metrics)
        state_hold_score = self._semantic_state_hold_score(candidate, target_state, seed_solution)
        memory_closeness_score = self._semantic_memory_closeness(candidate, seed_solution)
        wrist_motion_penalty = self._semantic_wrist_motion_penalty(candidate, current)
        upstream_motion_penalty = 0.0
        upright_score = clamp_value((alignment["upright_score"] + 1.0) * 0.5, 0.0, 1.0)
        total_score = (
            SEMANTIC_SCORE_WEIGHTS["changed_primary"] * changed_primary_score
            + SEMANTIC_SCORE_WEIGHTS["changed_secondary"] * changed_secondary_score
            + SEMANTIC_SCORE_WEIGHTS["state_hold"] * state_hold_score
            + SEMANTIC_SCORE_WEIGHTS["upright"] * upright_score
            + SEMANTIC_SCORE_WEIGHTS["memory"] * memory_closeness_score
            - SEMANTIC_SCORE_WEIGHTS["wrist_motion"] * wrist_motion_penalty
            - SEMANTIC_SCORE_WEIGHTS["upstream_motion"] * upstream_motion_penalty
        )
        accepted = validation["passed"]
        fallback_ok = (
            changed_primary_score >= SEMANTIC_FALLBACK_ALIGNMENT_FLOOR
            and changed_secondary_score >= SEMANTIC_FALLBACK_SECONDARY_FLOOR
        )
        return {
            "candidate": candidate,
            "alignment": alignment,
            "validation": validation,
            "changed_primary_score": changed_primary_score,
            "changed_secondary_score": changed_secondary_score,
            "state_hold_score": state_hold_score,
            "memory_closeness_score": memory_closeness_score,
            "upright_score": upright_score,
            "wrist_motion_penalty": wrist_motion_penalty,
            "upstream_motion_penalty": upstream_motion_penalty,
            "joint_0_mode": "avoid",
            "accepted": accepted,
            "fallback_ok": fallback_ok,
            "rank_key": (
                1 if accepted else 0,
                round(changed_primary_score, 6),
                round(changed_secondary_score, 6),
                round(state_hold_score, 6),
                round(memory_closeness_score, 6),
                round(upright_score, 6),
                -round(wrist_motion_penalty, 6),
            ),
            "total_score": total_score,
        }

    def _semantic_goal_constraints(
        self,
        target_state: dict,
        candidate: dict[str, float],
        tolerance_scale: float = 1.0,
    ) -> list[JointConstraint]:
        joint_tolerances = {
            "joint_3": 0.10,
            "joint_4": 0.10,
            "joint_5": 0.22,
        }
        if "horizontal" in target_state["requested_stages"]:
            joint_tolerances["joint_3"] = SEMANTIC_HORIZONTAL_PROFILES[target_state["horizontal"]]["goal_tolerance"]
        if "vertical" in target_state["requested_stages"]:
            joint_tolerances["joint_4"] = SEMANTIC_VERTICAL_PROFILES[target_state["vertical"]]["goal_tolerance"]
        constraints: list[JointConstraint] = []
        for joint_name in WRIST_SEMANTIC_JOINTS:
            tolerance = min(joint_tolerances[joint_name] * tolerance_scale, SEMANTIC_MAX_GOAL_TOLERANCE)
            constraints.append(joint_constraint(joint_name, candidate[joint_name], tolerance))
        return constraints

    def _semantic_upstream_hold_constraints(self, helper_mode: str) -> list[JointConstraint]:
        constraints: list[JointConstraint] = []
        tolerances = SEMANTIC_UPSTREAM_TOLERANCES[helper_mode]
        for joint_name in SEMANTIC_UPSTREAM_JOINTS:
            current_value = self.joints.get(joint_name, 0.0)
            constraints.append(joint_constraint(joint_name, current_value, tolerances[joint_name]))
        return constraints

    def _semantic_candidate_search(self, target_state: dict):
        if self._semantic_model is None:
            return None

        current_transform = self._tf_transform(SEMANTIC_REFERENCE_FRAME, LINK_NAME)
        if current_transform is None:
            return None
        current_link6_q = self._semantic_transform_quaternion(current_transform)
        current_alignment = self._semantic_alignment_from_quaternion(current_link6_q)
        context = self._semantic_prediction_context()
        if context is None or context["current_model_q"] is None:
            return None

        if context["missing_joint_states"]:
            self._log(
                "warn",
                "Semantic search missing joint states for "
                + ", ".join(context["missing_joint_states"])
                + "; using 0 deg defaults where required",
            )

        seed_solution, current, memory_source = self._semantic_memory_seed()
        samples = self._semantic_candidate_samples(target_state, seed_solution, current)
        current_model_alignment = self._semantic_alignment_from_quaternion(context["current_model_q"])
        current_model_tf_error_deg = self._semantic_quat_angle_deg(context["current_model_q"], current_link6_q)

        results = []
        for joint_3 in samples["joint_3"]:
            for joint_4 in samples["joint_4"]:
                for joint_5 in samples["joint_5"]:
                    candidate = {"joint_3": joint_3, "joint_4": joint_4, "joint_5": joint_5}
                    predicted_q = self._semantic_predict_candidate_quaternion(candidate, context)
                    if predicted_q is None:
                        continue
                    alignment = self._semantic_alignment_from_quaternion(predicted_q)
                    results.append(
                        self._semantic_candidate_result(
                            candidate,
                            alignment,
                            target_state,
                            current,
                            seed_solution,
                        )
                    )

        if not results:
            return None

        results.sort(key=lambda item: (item["rank_key"], item["total_score"]), reverse=True)
        accepted = [result for result in results if result["accepted"]]
        fallback = [result for result in results if result["fallback_ok"]]
        winner = accepted[0] if accepted else (fallback[0] if fallback else results[0])
        alternates = []
        seen = {tuple(round(winner["candidate"][joint_name], 6) for joint_name in WRIST_SEMANTIC_JOINTS)}
        for result in accepted[1:] + fallback[1:] + results[1:]:
            signature = tuple(round(result["candidate"][joint_name], 6) for joint_name in WRIST_SEMANTIC_JOINTS)
            if signature in seen:
                continue
            seen.add(signature)
            alternates.append(result)
            if len(alternates) >= max(0, SEMANTIC_MAX_PLANNER_ATTEMPTS - 2):
                break

        return {
            "semantic": target_state["command"],
            "target_state": target_state,
            "strategy": (
                f"two-stage semantic solve: vertical={target_state['vertical']} via joint_4, "
                f"horizontal={target_state['horizontal']} via joint_3, joint_5 for roll cleanup"
            ),
            "prediction_source": context["source"],
            "memory_source": memory_source,
            "sample_summary": self._semantic_sample_summary(samples),
            "candidate_count": len(results),
            "accepted_count": len(accepted),
            "fallback_count": len(fallback),
            "current_alignment": current_alignment,
            "current_model_alignment": current_model_alignment,
            "current_model_tf_error_deg": current_model_tf_error_deg,
            "current_wrist": current,
            "seed_solution": seed_solution,
            "winner": winner,
            "alternates": alternates,
            "selection_mode": "accepted" if accepted else ("fallback" if fallback else "rejected"),
            "sendable": bool(accepted or fallback),
            "rejection_reason": None if (accepted or fallback) else "no candidate met tightened primary and plane thresholds",
        }

    def _semantic_build_attempts(self, search: dict) -> list[dict]:
        attempts = [
            {
                "candidate_result": search["winner"],
                "tolerance_scale": 1.0,
                "label": "primary_tight",
                "upstream_mode": "tight",
            },
            {
                "candidate_result": search["winner"],
                "tolerance_scale": SEMANTIC_RELAXED_TOLERANCE_SCALE,
                "label": "primary_relaxed",
                "upstream_mode": "relaxed",
            },
        ]
        if "horizontal" in search["target_state"]["requested_stages"]:
            attempts.append(
                {
                    "candidate_result": search["winner"],
                    "tolerance_scale": SEMANTIC_RELAXED_TOLERANCE_SCALE,
                    "label": "joint0_helper_if_needed",
                    "upstream_mode": "helper",
                }
            )
        for alternate in search["alternates"]:
            attempts.append(
                {
                    "candidate_result": alternate,
                    "tolerance_scale": SEMANTIC_RELAXED_TOLERANCE_SCALE,
                    "label": "alternate_relaxed",
                    "upstream_mode": "relaxed",
                }
            )
            if len(attempts) >= SEMANTIC_MAX_PLANNER_ATTEMPTS:
                break
        return attempts

    def _semantic_joint_text(self, candidate: dict[str, float]) -> str:
        return ", ".join(
            f"{joint_name}={math.degrees(candidate[joint_name]):+.1f} deg" for joint_name in WRIST_SEMANTIC_JOINTS
        )

    def _dispatch_semantic_attempt(self, search: dict, attempt_index: int):
        attempt = search["attempts"][attempt_index]
        candidate_result = attempt["candidate_result"]
        constraints = Constraints()
        constraints.joint_constraints.extend(
            self._semantic_goal_constraints(
                search["target_state"],
                candidate_result["candidate"],
                tolerance_scale=attempt["tolerance_scale"],
            )
        )
        constraints.joint_constraints.extend(self._semantic_upstream_hold_constraints(attempt["upstream_mode"]))
        joint_text = self._semantic_joint_text(candidate_result["candidate"])
        self.last_semantic_validation = (
            f"{search['semantic']}: pending ({attempt_index + 1}/{len(search['attempts'])}, {attempt['label']})"
        )
        self.last_semantic_skip = "none"
        self._log(
            "info",
            "Sending semantic joint goal: "
            f"button={search['semantic']}, goal_type=semantic_joint_goal, "
            f"attempt={attempt_index + 1}/{len(search['attempts'])}, attempt_mode={attempt['label']}, "
            f"target_vertical={search['target_state']['vertical']}, target_horizontal={search['target_state']['horizontal']}, "
            f"changed={self._semantic_stage_description(search['target_state'])}, "
            f"memory_source={search['memory_source']}, preferred_joints=joint_4/joint_3/joint_5, "
            f"joint_0={'helper_allowed' if attempt['upstream_mode'] == 'helper' else 'held'} , "
            "position_constraint=off, orientation_constraint=off, "
            f"joint_constraints={', '.join(WRIST_SEMANTIC_JOINTS + SEMANTIC_UPSTREAM_JOINTS)}, "
            f"tolerance_scale={attempt['tolerance_scale']:.2f}, joint_targets={joint_text}"
        )
        self._send_constraints(
            constraints,
            prefer_distal_joints=True,
            goal_context={
                "type": "semantic_joint_goal",
                "semantic": search["semantic"],
                "target_state": search["target_state"],
                "attempt_index": attempt_index,
                "attempts": search["attempts"],
                "search": search,
                "candidate_result": candidate_result,
            },
        )

    def _maybe_retry_active_semantic_goal(self, reason: str) -> bool:
        context = self._active_goal_context
        if context is None or context.get("type") != "semantic_joint_goal":
            return False
        next_attempt_index = context["attempt_index"] + 1
        attempts = context["attempts"]
        if next_attempt_index >= len(attempts):
            return False
        next_attempt = attempts[next_attempt_index]
        next_candidate = next_attempt["candidate_result"]["candidate"]
        self._log(
            "warn",
            "Semantic goal retry: "
            f"{reason}; next_attempt={next_attempt_index + 1}/{len(attempts)}, "
            f"mode={next_attempt['label']}, "
            f"joint_0_mode={next_attempt['upstream_mode']}, "
            f"joints={self._semantic_joint_text(next_candidate)}",
        )
        self._dispatch_semantic_attempt(context["search"], next_attempt_index)
        return True

    def _schedule_semantic_validation(self, context: dict):
        if self._semantic_validation_timer is not None:
            try:
                self._semantic_validation_timer.cancel()
            except Exception:
                pass

        def _validate():
            alignment = self._semantic_alignment_state()
            if alignment is None:
                self.last_semantic_validation = f"{context['semantic']}: TF validation unavailable"
                self._log(
                    "warn",
                    f"Semantic post-command validation unavailable: no {SEMANTIC_REFERENCE_FRAME}->{LINK_NAME} TF",
                )
                return
            validation = self._semantic_validation_metrics(context["target_state"], alignment)
            current, _ = self._semantic_joint_state_snapshot(WRIST_SEMANTIC_JOINTS)
            if validation["passed"]:
                self.semantic_state["vertical_mode"] = context["target_state"]["vertical"]
                self.semantic_state["horizontal_mode"] = context["target_state"]["horizontal"]
                self.semantic_state["last_successful_state"] = {
                    "vertical": context["target_state"]["vertical"],
                    "horizontal": context["target_state"]["horizontal"],
                }
                self.semantic_state["last_successful_candidate"] = dict(current)
            self.semantic_state["last_successful_validation"] = (
                f"primary={validation['primary_score']:.3f}, secondary={validation['secondary_score']:.3f}"
            )
            self.last_semantic_validation = (
                f"{context['semantic']}: vertical={context['target_state']['vertical']} "
                f"({validation['vertical']['primary']:.3f}/{validation['vertical']['secondary']:.3f}), "
                f"horizontal={context['target_state']['horizontal']} "
                f"({validation['horizontal']['primary']:.3f}/{validation['horizontal']['secondary']:.3f}), "
                f"camera={alignment['upright_state']}, ok={'yes' if validation['passed'] else 'no'}"
            )
            self._log(
                "info",
                "Semantic post-command validation: "
                f"requested_button={context['semantic']}, target_vertical={context['target_state']['vertical']}, "
                f"target_horizontal={context['target_state']['horizontal']}, "
                f"achieved={alignment['semantic']} via tool {alignment['axis_name']}, "
                f"vertical_primary={validation['vertical']['primary']:.3f}, vertical_plane={validation['vertical']['secondary']:.3f}, "
                f"horizontal_primary={validation['horizontal']['primary']:.3f}, horizontal_plane={validation['horizontal']['secondary']:.3f}, "
                f"camera={alignment['upright_state']} ({alignment['upright_score']:+.3f}), "
                f"result={'accepted' if validation['passed'] else 'sloppy'}"
            )

        self._semantic_validation_timer = threading.Timer(SEMANTIC_VALIDATION_DELAY_SEC, _validate)
        self._semantic_validation_timer.daemon = True
        self._semantic_validation_timer.start()

    def apply_semantic_tool_orientation(self, semantic: str, send_goal: bool = True):
        if semantic not in SEMANTIC_COMMAND_TARGETS:
            self._log("error", f"Unknown semantic direction: {semantic}")
            return False
        if self._semantic_model is None:
            self._log("error", "Semantic command unavailable: URDF-backed kinematic model did not load")
            return False
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return False

        current_alignment = self._semantic_alignment_state()
        self._semantic_seed_state_if_needed(current_alignment)
        target_state = self._semantic_target_state(semantic)
        if target_state is None:
            self._log("error", f"Unknown semantic direction: {semantic}")
            return False
        self.semantic_state["last_requested_command"] = semantic

        already_satisfied = self._semantic_already_satisfied(target_state, current_alignment)
        if already_satisfied is not None:
            current_wrist, _ = self._semantic_joint_state_snapshot(WRIST_SEMANTIC_JOINTS)
            self.semantic_state["vertical_mode"] = target_state["vertical"]
            self.semantic_state["horizontal_mode"] = target_state["horizontal"]
            self.semantic_state["last_successful_state"] = {
                "vertical": target_state["vertical"],
                "horizontal": target_state["horizontal"],
            }
            self.semantic_state["last_successful_candidate"] = dict(current_wrist)
            self.semantic_state["last_successful_validation"] = (
                f"primary={already_satisfied['primary_score']:.3f}, secondary={already_satisfied['secondary_score']:.3f}"
            )
            self.last_semantic_skip = (
                f"{semantic}: already satisfied, skipped motion "
                f"(vertical={target_state['vertical']}, horizontal={target_state['horizontal']})"
            )
            self.last_semantic_validation = self.semantic_state["last_successful_validation"]
            self.last_semantic_strategy = (
                f"{semantic}: skipped, target vertical={target_state['vertical']}, "
                f"horizontal={target_state['horizontal']}, changed={self._semantic_stage_description(target_state)}"
            )
            self._log(
                "info",
                "Semantic command skipped: "
                f"button={semantic}, target_vertical={target_state['vertical']}, "
                f"target_horizontal={target_state['horizontal']}, changed={self._semantic_stage_description(target_state)}, "
                f"reason=already within tightened semantic thresholds",
            )
            return True

        search = self._semantic_candidate_search(target_state)
        if search is None:
            self._log(
                "error",
                f"No {SEMANTIC_REFERENCE_FRAME}->{LINK_NAME} transform available for semantic candidate search",
            )
            return False
        if not search["sendable"]:
            self._log(
                "error",
                f"Semantic candidate search failed for {semantic}: {search['rejection_reason']}",
            )
            self.last_semantic_validation = f"{semantic}: rejected ({search['rejection_reason']})"
            return False

        winner = search["winner"]
        joint_text = self._semantic_joint_text(winner["candidate"])
        fallback_note = ""
        if search["selection_mode"] != "accepted":
            fallback_note = f"; fallback={search['selection_mode']}"
        self.last_semantic_strategy = (
            f"{semantic}: vertical={target_state['vertical']}, horizontal={target_state['horizontal']}, "
            f"changed={self._semantic_stage_description(target_state)} "
            f"[{joint_text}; primary={winner['changed_primary_score']:.3f}; plane={winner['changed_secondary_score']:.3f}; "
            f"memory={winner['memory_closeness_score']:.3f}; candidates={search['candidate_count']}; "
            f"accepted={search['accepted_count']}{fallback_note}]"
        )
        self.last_semantic_skip = "none"

        self._log(
            "info",
            "Semantic command requested: "
            f"button={semantic}, semantic_reference={SEMANTIC_REFERENCE_FRAME} +X, "
            f"target_vertical={target_state['vertical']}, target_horizontal={target_state['horizontal']}, "
            f"changed={self._semantic_stage_description(target_state)}, "
            f"current_alignment={search['current_alignment']['semantic']} via tool {search['current_alignment']['axis_name']} "
            f"[{search['current_alignment']['axis_vector'][0]:+.3f}, {search['current_alignment']['axis_vector'][1]:+.3f}, "
            f"{search['current_alignment']['axis_vector'][2]:+.3f}] "
            f"(score={search['current_alignment']['score']:.3f}, camera={search['current_alignment']['upright_state']})"
        )
        self._log(
            "info",
            "Semantic candidate search: "
            f"evaluation_source={search['prediction_source']}, "
            f"candidate_ranges={search['sample_summary']}, "
            f"memory_source={search['memory_source']}, "
            f"candidates_evaluated={search['candidate_count']}, "
            f"accepted={search['accepted_count']}, fallback={search['fallback_count']}, "
            f"current_fk_alignment={search['current_model_alignment']['semantic']} via tool {search['current_model_alignment']['axis_name']} "
            f"(tf_delta={search['current_model_tf_error_deg']:.2f} deg)"
        )
        if search["current_model_tf_error_deg"] > SEMANTIC_FK_TF_WARN_DEG:
            self._log(
                "warn",
                "Semantic FK/TF mismatch warning: "
                f"current prediction differs from live TF by {search['current_model_tf_error_deg']:.2f} deg",
            )
        if search["selection_mode"] != "accepted":
            self._log(
                "warn",
                "Semantic candidate fallback: "
                f"no candidate met tightened primary/plane thresholds; "
                f"using best available candidate at primary={winner['changed_primary_score']:.3f}, "
                f"plane={winner['changed_secondary_score']:.3f}",
            )

        self._log(
            "info",
            "Semantic candidate winner: "
            f"strategy={search['strategy']}, selection_mode={search['selection_mode']}, chosen_joints={joint_text}, "
            f"predicted_semantic={winner['alignment']['semantic']} via tool {winner['alignment']['axis_name']}, "
            f"primary_axis_score={winner['changed_primary_score']:.3f}, "
            f"secondary_plane_score={winner['changed_secondary_score']:.3f}, "
            f"upright_score={winner['upright_score']:.3f}, memory_closeness_score={winner['memory_closeness_score']:.3f}, "
            f"state_hold_score={winner['state_hold_score']:.3f}, upstream_motion_penalty={winner['upstream_motion_penalty']:.3f}, "
            f"wrist_motion_penalty={winner['wrist_motion_penalty']:.3f}, joint_0={winner['joint_0_mode']}, total={winner['total_score']:.3f}"
        )

        if not send_goal:
            return True

        search["attempts"] = self._semantic_build_attempts(search)
        self.done.clear()
        self._dispatch_semantic_attempt(search, 0)
        return True

    # Historical name kept for GUI/call-site compatibility. The semantic layer
    # now classifies whichever tool axis should align with link_1 +X instead of
    # assuming every command is only about tool +Z.
    def align_link6_z_to_semantic(self, semantic: str, send_goal: bool = True):
        return self.apply_semantic_tool_orientation(semantic, send_goal=send_goal)

    def capture_current_orientation(self) -> bool:
        # Capture orientation of the end-effector expressed in the current reference frame
        transform = self._tf_transform(self.reference_frame, LINK_NAME)
        if transform is None:
            return False
        rotation = transform.rotation
        self.target_orientation = (rotation.x, rotation.y, rotation.z, rotation.w)
        roll, pitch, yaw = euler_from_quat(rotation.x, rotation.y, rotation.z, rotation.w)
        self.target_orientation_rpy_deg = [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
        self._log(
            "info",
            "Captured current orientation -> "
            f"roll={self.target_orientation_rpy_deg[0]:.1f} deg, "
            f"pitch={self.target_orientation_rpy_deg[1]:.1f} deg, "
            f"yaw={self.target_orientation_rpy_deg[2]:.1f} deg",
        )
        return True

    def set_orientation_from_rpy_deg(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        self.target_orientation_rpy_deg = [roll_deg, pitch_deg, yaw_deg]
        self.target_orientation = quat_from_euler(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )
        self.last_semantic_strategy = "none"
        self.last_semantic_validation = "none"
        self.last_semantic_skip = "none"
        self._log(
            "info",
            f"Orientation target set -> roll={roll_deg:.1f} deg, pitch={pitch_deg:.1f} deg, yaw={yaw_deg:.1f} deg",
        )

    def set_maintain_orientation(self, enabled: bool):
        self.maintain_orientation = enabled
        self._log("info", f"Maintain orientation: {'ON' if enabled else 'OFF'}")

    def set_reference_frame(self, frame: str):
        self.reference_frame = frame
        self._log('info', f'Set reference frame -> {frame}')
        # Try immediate capture; if unavailable, schedule a short retry
        ok = False
        try:
            ok = self.capture_current_orientation()
        except Exception:
            ok = False
        if not ok:
            if self._ref_capture_timer is not None:
                try:
                    self._ref_capture_timer.cancel()
                except Exception:
                    pass
            def _retry():
                try:
                    if self.capture_current_orientation():
                        self._log('info', 'Captured orientation after retry')
                except Exception:
                    pass
            self._ref_capture_timer = threading.Timer(0.5, _retry)
            self._ref_capture_timer.daemon = True
            self._ref_capture_timer.start()

    def status_text(self):
        mode = "fixed orientation" if self.maintain_orientation else "position only"
        lines = [f"Mode: {mode}", "Joints:"]
        for name in DISPLAY_JOINTS:
            value = self.joints.get(name, float("nan"))
            lines.append(f"  {name}  {value:+.4f} rad  ({math.degrees(value):+.1f} deg)")
        for name in ["left_gripper", "right_gripper"]:
            if name in self.joints:
                lines.append(f"  {name}  {self.joints[name]:+.4f} m")
        lines.append(
            "Target orientation: "
            f"roll={self.target_orientation_rpy_deg[0]:.1f} deg, "
            f"pitch={self.target_orientation_rpy_deg[1]:.1f} deg, "
            f"yaw={self.target_orientation_rpy_deg[2]:.1f} deg"
        )
        lines.append(
            "Semantic state: "
            f"vertical={self._semantic_mode_label('vertical', self.semantic_state['vertical_mode'])}, "
            f"horizontal={self._semantic_mode_label('horizontal', self.semantic_state['horizontal_mode'])}"
        )
        last_successful_state = self.semantic_state["last_successful_state"]
        if last_successful_state is None:
            lines.append("Last successful semantic state: none")
        else:
            lines.append(
                "Last successful semantic state: "
                f"vertical={self._semantic_mode_label('vertical', last_successful_state['vertical'])}, "
                f"horizontal={self._semantic_mode_label('horizontal', last_successful_state['horizontal'])}"
            )
        lines.append(f"Semantic strategy: {self.last_semantic_strategy}")
        lines.append(f"Semantic validation: {self.last_semantic_validation}")
        lines.append(f"Semantic skip: {self.last_semantic_skip}")
        lines.append(f"Semantic reference: {SEMANTIC_REFERENCE_FRAME} +X")
        world_transform = self._tf_transform(self.reference_frame, LINK_NAME)
        if world_transform is not None:
            position = world_transform.translation
            lines.append(f"EE pose ({self.reference_frame}): x={position.x:.4f} y={position.y:.4f} z={position.z:.4f}")
            roll, pitch, yaw = euler_from_quat(
                world_transform.rotation.x,
                world_transform.rotation.y,
                world_transform.rotation.z,
                world_transform.rotation.w,
            )
            lines.append(
                "EE orientation: "
                f"roll={math.degrees(roll):.1f} deg, pitch={math.degrees(pitch):.1f} deg, yaw={math.degrees(yaw):.1f} deg"
            )
        alignment = self._semantic_alignment_state()
        if alignment is not None:
            lines.append(
                f"Semantic alignment: {alignment['semantic']} via tool {alignment['axis_name']} -> {SEMANTIC_REFERENCE_FRAME} +X "
                f"[{alignment['axis_vector'][0]:+.3f}, {alignment['axis_vector'][1]:+.3f}, {alignment['axis_vector'][2]:+.3f}] "
                f"(score {alignment['score']:.3f})"
            )
            lines.append(
                f"Camera upright note: {alignment['upright_state']} "
                f"(tool +Y vs {SEMANTIC_REFERENCE_FRAME} +Z score {alignment['upright_score']:+.3f})"
            )
            for axis_name in ["+X", "+Y", "+Z"]:
                axis_vector = alignment['axis_vectors'][axis_name]
                lines.append(
                    f"Tool {axis_name} (in {SEMANTIC_REFERENCE_FRAME}): "
                    f"[{axis_vector[0]:+.3f}, {axis_vector[1]:+.3f}, {axis_vector[2]:+.3f}]"
                )
        return "\n".join(lines)

    def print_status(self):
        self._log("info", self.status_text())

    def _position_constraint(self, x, y, z, tol=POSITION_TOL):
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.reference_frame
        position_constraint.link_name = LINK_NAME
        position_constraint.weight = 1.0
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [tol, tol, tol]
        position_constraint.constraint_region.primitives.append(box)
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.w = 1.0
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        return position_constraint

    def _orientation_constraint(self):
        qx, qy, qz, qw = self.target_orientation
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.reference_frame
        orientation_constraint.link_name = LINK_NAME
        orientation_constraint.orientation.x = qx
        orientation_constraint.orientation.y = qy
        orientation_constraint.orientation.z = qz
        orientation_constraint.orientation.w = qw
        orientation_constraint.absolute_x_axis_tolerance = ORIENTATION_TOL
        orientation_constraint.absolute_y_axis_tolerance = ORIENTATION_TOL
        orientation_constraint.absolute_z_axis_tolerance = ORIENTATION_TOL
        orientation_constraint.weight = 1.0
        return orientation_constraint

    def move_xyz(self, dx, dy, dz, label):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        transform = self._tf_transform(self.reference_frame, LINK_NAME)
        if transform is None:
            return
        target_x = transform.translation.x + dx
        target_y = transform.translation.y + dy
        target_z = transform.translation.z + dz
        self._log(
            "info",
            f"XYZ {math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) * 100.0:.1f}cm {label}"
            + (" [fixed orientation]" if self.maintain_orientation else " [position only]"),
        )
        self._send_pose_goal(target_x, target_y, target_z)

    def apply_orientation_here(self):
        transform = self._tf_transform(self.reference_frame, LINK_NAME)
        if transform is None:
            return
        self._log("info", "Applying orientation target at current position")
        self._send_pose_goal(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            force_orientation=True,
        )

    def go_home(self):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        self.done.clear()
        constraints = Constraints()
        for joint_name, value in HOME_JOINTS.items():
            constraints.joint_constraints.append(joint_constraint(joint_name, value, 0.03))
        self._log("info", "HOME -> all joints to 0 deg")
        self._send_constraints(constraints)

    def set_gripper(self, opening: float):
        opening = max(GRIPPER_MIN, min(GRIPPER_MAX, opening))
        if not self.hand_done.is_set():
            self._log("warn", "Gripper still executing.")
            return
        if not self._hand_client.wait_for_server(timeout_sec=1.0):
            self._log("error", "Hand controller action server unavailable")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_gripper", "right_gripper"]
        point = JointTrajectoryPoint()
        point.positions = [GRIPPER_MAX - opening, opening]
        point.time_from_start = BuiltinDuration(sec=0, nanosec=int(GRIPPER_TRAJECTORY_SECONDS * 1_000_000_000))
        goal.trajectory.points = [point]

        self._log("info", f"Gripper opening -> {opening:.3f} m")
        self.hand_done.clear()
        self._hand_client.send_goal_async(goal).add_done_callback(self._on_hand_goal)

    def _on_hand_goal(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._log("warn", "Gripper goal rejected.")
            self.hand_done.set()
            return
        goal_handle.get_result_async().add_done_callback(self._on_hand_result)

    def _on_hand_result(self, future):
        result = future.result()
        if result is None:
            self._log("warn", "Gripper command failed.")
            self.hand_done.set()
            return
        self._log("info", "Gripper command done.")
        self.hand_done.set()

    def _send_pose_goal(
        self,
        x: float,
        y: float,
        z: float,
        prefer_distal_joints: bool = False,
        force_orientation: bool = False,
    ):
        if not self.done.is_set():
            self._log("warn", "Still executing.")
            return
        self.done.clear()
        constraints = Constraints()
        constraints.position_constraints.append(self._position_constraint(x, y, z))
        # Explicit orientation commands must still constrain orientation even if
        # free-orientation jogging mode is enabled.
        if self.maintain_orientation or force_orientation:
            constraints.orientation_constraints.append(self._orientation_constraint())
        # attach joint preference hint if requested
        self._send_constraints(constraints, prefer_distal_joints=prefer_distal_joints)

    def _send_constraints(
        self,
        constraints: Constraints,
        prefer_distal_joints: bool = False,
        goal_context: dict | None = None,
    ):
        self._active_goal_context = goal_context
        explicit_joint_names = {jc.joint_name for jc in constraints.joint_constraints}
        # Debug: log constraint frames and basic info
        try:
            frame_info = []
            orientation_included = bool(constraints.orientation_constraints)
            for pc in constraints.position_constraints:
                frame_info.append(f"pos(frame={pc.header.frame_id})")
            for oc in constraints.orientation_constraints:
                frame_info.append(f"orient(frame={oc.header.frame_id})")
            if explicit_joint_names:
                frame_info.append("joints=" + ",".join(sorted(explicit_joint_names)))
            frame_info.append(f"orientation={'on' if orientation_included else 'off'}")
            if prefer_distal_joints:
                frame_info.append("joint_bias=distal")
            if goal_context is not None:
                frame_info.append(f"goal={goal_context.get('type', 'unspecified')}")
            self._log("info", "Sending goal with constraints: " + ", ".join(frame_info))
        except Exception:
            pass
        # If requested, add joint constraints that prefer moving distal joints first
        if prefer_distal_joints:
            # tolerance mapping from joint index -> allowed movement (rad)
            # larger tolerance => planner may move that joint more easily
            tol_map = {5: 0.6, 4: 0.45, 3: 0.3, 2: 0.15, 1: 0.07, 0: 0.03}
            # add joint constraints based on current joint positions
            for name in DISPLAY_JOINTS:
                if name in explicit_joint_names:
                    continue
                if name in self.joints:
                    idx = int(name.split('_')[-1])
                    tol = tol_map.get(idx, 0.05)
                    jc = joint_constraint(name, self.joints[name], tol)
                    constraints.joint_constraints.append(jc)
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP_NAME
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 10
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.goal_constraints.append(constraints)
        # Wait for action server (log if unavailable)
        if not self._client.wait_for_server(timeout_sec=2.0):
            self._log("error", "Move action server unavailable when sending goal")
            if goal_context is not None and goal_context.get("type") == "semantic_joint_goal":
                self.last_semantic_validation = f"{goal_context['semantic']}: move action server unavailable"
            self._active_goal_context = None
            self.done.set()
            return
        self._client.send_goal_async(goal).add_done_callback(self._on_goal)

    def _on_goal(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            if self._maybe_retry_active_semantic_goal("planner rejected goal"):
                return
            self._log("warn", "Goal rejected.")
            if self._active_goal_context is not None and self._active_goal_context.get("type") == "semantic_joint_goal":
                self.last_semantic_validation = f"{self._active_goal_context['semantic']}: goal rejected"
            self._active_goal_context = None
            self.done.set()
            return
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        if result is None:
            if self._maybe_retry_active_semantic_goal("execution result missing"):
                return
            self._log("warn", "Failed (no result)")
            if self._active_goal_context is not None and self._active_goal_context.get("type") == "semantic_joint_goal":
                self.last_semantic_validation = f"{self._active_goal_context['semantic']}: execution result missing"
            self._active_goal_context = None
            self.done.set()
            return

        value = result.result.error_code.val
        if value != 1 and self._maybe_retry_active_semantic_goal(f"failed with code {value}"):
            return

        self._log("info", "Done." if value == 1 else f"Failed (code {value})")
        if self._active_goal_context is not None and self._active_goal_context.get("type") == "semantic_joint_goal":
            if value == 1:
                self._schedule_semantic_validation(dict(self._active_goal_context))
            else:
                self.last_semantic_validation = f"{self._active_goal_context['semantic']}: failed (code {value})"
        self._active_goal_context = None
        self.done.set()


class TeleopGui:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.log_queue = Queue()
        self.ref_frame_var = tk.StringVar(value=self.node.reference_frame)
        self.xyz_step_var = tk.StringVar(value=str(DEFAULT_CM))
        self.hold_var = tk.BooleanVar(value=True)
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.gripper_var = tk.DoubleVar(value=GRIPPER_OPEN_BUTTON)
        self.mode_var = tk.StringVar(value="Mode: fixed orientation")
        self.exec_var = tk.StringVar(value="Planner: idle")
        self.joint_var = tk.StringVar(value="Waiting for joint states...")

        self.node.log_callback = self.enqueue_log

        self.root.title("SixDOF Pose Teleop")
        self.root.geometry("1020x820")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_ui()
        self.root.after(100, self._drain_logs)
        self.root.after(250, self._refresh_status)

    def enqueue_log(self, level, text):
        self.log_queue.put((level, text))

    def _build_ui(self):
        main = tk.Frame(self.root, padx=12, pady=12)
        main.pack(fill=tk.BOTH, expand=True)

        header = tk.Label(main, text="SixDOF Pose Teleop", font=("TkDefaultFont", 16, "bold"))
        header.pack(anchor=tk.W)

        info = tk.Label(
            main,
            text="Jog XYZ in the current reference frame. Toggle fixed orientation on or off. Use presets or custom RPY to test full 6DOF pose IK.",
            justify=tk.LEFT,
        )
        info.pack(anchor=tk.W, pady=(4, 10))

        status = tk.Frame(main)
        status.pack(fill=tk.X, pady=(0, 12))
        tk.Label(status, textvariable=self.mode_var, width=34, anchor=tk.W).pack(side=tk.LEFT)
        tk.Label(status, textvariable=self.exec_var, width=18, anchor=tk.W).pack(side=tk.LEFT, padx=(12, 0))
        # Reference frame selector
        ref_frame_frame = tk.Frame(status)
        ref_frame_frame.pack(side=tk.RIGHT)
        tk.Label(ref_frame_frame, text="Reference Frame:").pack(side=tk.LEFT)
        tk.OptionMenu(ref_frame_frame, self.ref_frame_var, "world", "ee_ref").pack(side=tk.LEFT)
        tk.Button(ref_frame_frame, text="Set", command=lambda: self.node.set_reference_frame(self.ref_frame_var.get())).pack(side=tk.LEFT, padx=(6,0))

        controls = tk.Frame(main)
        controls.pack(fill=tk.X)

        xyz = tk.LabelFrame(controls, text="XYZ Move (cm)", padx=10, pady=10)
        xyz.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        tk.Label(xyz, text="Step").grid(row=0, column=0, sticky="w")
        tk.Entry(xyz, textvariable=self.xyz_step_var, width=8).grid(row=0, column=1, sticky="w")
        tk.Button(xyz, text="Back", width=8, command=lambda: self._move_xyz(+1, 0, 0, "Back")).grid(row=1, column=1, pady=4)
        tk.Button(xyz, text="Forward", width=8, command=lambda: self._move_xyz(-1, 0, 0, "Forward")).grid(row=3, column=1, pady=4)
        tk.Button(xyz, text="Left", width=8, command=lambda: self._move_xyz(0, -1, 0, "Left")).grid(row=2, column=0, padx=4)
        tk.Button(xyz, text="Right", width=8, command=lambda: self._move_xyz(0, +1, 0, "Right")).grid(row=2, column=2, padx=4)
        tk.Button(xyz, text="+Z", width=8, command=lambda: self._move_xyz(0, 0, +1, "+Z up")).grid(row=1, column=3, padx=(12, 0))
        tk.Button(xyz, text="-Z", width=8, command=lambda: self._move_xyz(0, 0, -1, "-Z down")).grid(row=3, column=3, padx=(12, 0))

        orient = tk.LabelFrame(controls, text="Orientation", padx=10, pady=10)
        orient.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 0))
        tk.Checkbutton(
            orient,
            text="Maintain fixed orientation",
            variable=self.hold_var,
            command=self._toggle_hold,
        ).grid(row=0, column=0, columnspan=4, sticky="w")
        tk.Button(orient, text="Capture Current", width=14, command=self._capture_current).grid(row=1, column=0, pady=4, sticky="w")
        tk.Button(orient, text="Apply Here", width=14, command=self.node.apply_orientation_here).grid(row=1, column=1, pady=4, sticky="w")
        tk.Button(orient, text="Look Forward", width=14, command=lambda: self._apply_preset("Look Forward")).grid(row=2, column=0, pady=4, sticky="w")
        tk.Button(orient, text="Look Down", width=14, command=lambda: self._apply_preset("Look Down")).grid(row=2, column=1, pady=4, sticky="w")
        tk.Button(orient, text="Look Up", width=14, command=lambda: self._apply_preset("Look Up")).grid(row=3, column=0, pady=4, sticky="w")
        tk.Button(orient, text="Look Right", width=14, command=lambda: self._apply_preset("Look Right")).grid(row=3, column=1, pady=4, sticky="w")
        tk.Button(orient, text="Look Left", width=14, command=lambda: self._apply_preset("Look Left")).grid(row=4, column=0, pady=4, sticky="w")
        tk.Button(orient, text="Apply RPY", width=14, command=self._apply_rpy).grid(row=4, column=1, pady=4, sticky="w")

        # Reference rotation controls: axis selector + custom degrees
        tk.Label(orient, text="Axis").grid(row=5, column=2, sticky="w")
        self.axis_var = tk.StringVar(value="Z")
        tk.OptionMenu(orient, self.axis_var, "X", "Y", "Z").grid(row=5, column=3, sticky="w")
        tk.Label(orient, text="Degrees").grid(row=6, column=2, sticky="w")
        self.deg_var = tk.StringVar(value="45")
        tk.Entry(orient, textvariable=self.deg_var, width=6).grid(row=6, column=3, sticky="w")
        tk.Button(orient, text="Rotate Ref", width=14, command=self._rotate_ref_custom).grid(row=7, column=2, columnspan=2, pady=4, sticky="w")

        tk.Label(orient, text="Roll").grid(row=5, column=0, sticky="w", pady=(8, 0))
        tk.Entry(orient, textvariable=self.roll_var, width=10).grid(row=5, column=1, sticky="w", pady=(8, 0))
        tk.Label(orient, text="Pitch").grid(row=6, column=0, sticky="w")
        tk.Entry(orient, textvariable=self.pitch_var, width=10).grid(row=6, column=1, sticky="w")
        tk.Label(orient, text="Yaw").grid(row=7, column=0, sticky="w")
        tk.Entry(orient, textvariable=self.yaw_var, width=10).grid(row=7, column=1, sticky="w")

        gripper = tk.LabelFrame(main, text="Gripper", padx=10, pady=10)
        gripper.pack(fill=tk.X, pady=(12, 12))
        tk.Scale(
            gripper,
            from_=GRIPPER_MIN,
            to=GRIPPER_MAX,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=400,
            variable=self.gripper_var,
            label="Opening (m)",
        ).pack(side=tk.LEFT, padx=(0, 12))
        buttons = tk.Frame(gripper)
        buttons.pack(side=tk.LEFT)
        tk.Button(buttons, text="Apply Slider", width=14, command=self._apply_gripper_slider).pack(pady=2)
        tk.Button(buttons, text="Open", width=14, command=self._open_gripper).pack(pady=2)
        tk.Button(buttons, text="Close", width=14, command=self._close_gripper).pack(pady=2)

        actions = tk.LabelFrame(main, text="Actions", padx=10, pady=10)
        actions.pack(fill=tk.X, pady=(0, 12))
        tk.Button(actions, text="Home", width=12, command=self.node.go_home).pack(side=tk.LEFT)
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
        self.mode_var.set("Mode: fixed orientation" if self.node.maintain_orientation else "Mode: position only")
        self.exec_var.set("Planner: busy" if not self.node.done.is_set() else "Planner: idle")
        values = []
        for name in DISPLAY_JOINTS:
            if name in self.node.joints:
                values.append(f"{name}: {math.degrees(self.node.joints[name]):+.1f} deg")
            else:
                values.append(f"{name}: n/a")
        for name in ["left_gripper", "right_gripper"]:
            if name in self.node.joints:
                values.append(f"{name}: {self.node.joints[name]:+.3f} m")
        values.append(
            "Target RPY: "
            f"{self.node.target_orientation_rpy_deg[0]:+.1f}, "
            f"{self.node.target_orientation_rpy_deg[1]:+.1f}, "
            f"{self.node.target_orientation_rpy_deg[2]:+.1f} deg"
        )
        values.append(
            "Semantic state: "
            f"vertical={self.node._semantic_mode_label('vertical', self.node.semantic_state['vertical_mode'])}, "
            f"horizontal={self.node._semantic_mode_label('horizontal', self.node.semantic_state['horizontal_mode'])}"
        )
        last_successful_state = self.node.semantic_state["last_successful_state"]
        if last_successful_state is None:
            values.append("Last successful semantic state: none")
        else:
            values.append(
                "Last successful semantic state: "
                f"vertical={self.node._semantic_mode_label('vertical', last_successful_state['vertical'])}, "
                f"horizontal={self.node._semantic_mode_label('horizontal', last_successful_state['horizontal'])}"
            )
        values.append(f"Semantic strategy: {self.node.last_semantic_strategy}")
        values.append(f"Semantic validation: {self.node.last_semantic_validation}")
        values.append(f"Semantic skip: {self.node.last_semantic_skip}")
        alignment = self.node._semantic_alignment_state()
        if alignment is not None:
            values.append(
                f"Semantic: {alignment['semantic']} via tool {alignment['axis_name']} -> {SEMANTIC_REFERENCE_FRAME} +X"
            )
            values.append(f"Camera: {alignment['upright_state']}")
        self.joint_var.set("\n".join(values))
        self.root.after(250, self._refresh_status)

    def _parse_float(self, value, label):
        try:
            return float(value)
        except ValueError:
            self.enqueue_log("error", f"Invalid {label}: {value}")
            return None

    def _move_xyz(self, x_sign, y_sign, z_sign, label):
        cm = self._parse_float(self.xyz_step_var.get(), "XYZ step")
        if cm is None:
            return
        metres = cm / 100.0
        self.node.move_xyz(x_sign * metres, y_sign * metres, z_sign * metres, label)

    def _toggle_hold(self):
        enabled = self.hold_var.get()
        self.node.set_maintain_orientation(enabled)
        if enabled and self.node.capture_current_orientation():
            self.roll_var.set(f"{self.node.target_orientation_rpy_deg[0]:.1f}")
            self.pitch_var.set(f"{self.node.target_orientation_rpy_deg[1]:.1f}")
            self.yaw_var.set(f"{self.node.target_orientation_rpy_deg[2]:.1f}")

    def _capture_current(self):
        if self.node.capture_current_orientation():
            self.roll_var.set(f"{self.node.target_orientation_rpy_deg[0]:.1f}")
            self.pitch_var.set(f"{self.node.target_orientation_rpy_deg[1]:.1f}")
            self.yaw_var.set(f"{self.node.target_orientation_rpy_deg[2]:.1f}")
            self.enqueue_log("info", "Captured current end-effector orientation into the target fields.")

    def _apply_preset(self, preset_name):
        # Map GUI presets to semantic directions and use alignment routine
        semantic_map = {
            'Look Forward': 'Forward',
            'Look Down': 'Down',
            'Look Up': 'Up',
            'Look Right': 'Right',
            'Look Left': 'Left',
        }
        if preset_name in semantic_map:
            sem = semantic_map[preset_name]
            ok = self.node.apply_semantic_tool_orientation(sem, send_goal=True)
            if ok:
                self.enqueue_log("info", f"Applied semantic preset: {preset_name} -> {sem}")
            else:
                self.enqueue_log("error", f"Failed to apply semantic preset: {preset_name}")
        else:
            # fallback to original RPY behavior
            roll_deg, pitch_deg, yaw_deg = ORIENTATION_PRESETS_DEG[preset_name]
            self.roll_var.set(f"{roll_deg:.1f}")
            self.pitch_var.set(f"{pitch_deg:.1f}")
            self.yaw_var.set(f"{yaw_deg:.1f}")
            self.node.set_orientation_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)
            self.enqueue_log("info", f"Applied preset: {preset_name}")

    def _apply_rpy(self):
        roll_deg = self._parse_float(self.roll_var.get(), "roll")
        pitch_deg = self._parse_float(self.pitch_var.get(), "pitch")
        yaw_deg = self._parse_float(self.yaw_var.get(), "yaw")
        if None in (roll_deg, pitch_deg, yaw_deg):
            return
        self.node.set_orientation_from_rpy_deg(roll_deg, pitch_deg, yaw_deg)

    def _rotate_ref_custom(self):
        try:
            deg = float(self.deg_var.get())
        except ValueError:
            self.enqueue_log("error", f"Invalid degrees: {self.deg_var.get()}")
            return
        axis = self.axis_var.get()
        self.node.rotate_reference(axis, deg)

    def _apply_gripper_slider(self):
        self.node.set_gripper(self.gripper_var.get())

    def _open_gripper(self):
        self.gripper_var.set(GRIPPER_OPEN_BUTTON)
        self.node.set_gripper(GRIPPER_OPEN_BUTTON)

    def _close_gripper(self):
        self.gripper_var.set(GRIPPER_CLOSE_BUTTON)
        self.node.set_gripper(GRIPPER_CLOSE_BUTTON)

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
        app._capture_current()
        app._append_log("[INFO] 6DOF pose teleop GUI ready.")
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

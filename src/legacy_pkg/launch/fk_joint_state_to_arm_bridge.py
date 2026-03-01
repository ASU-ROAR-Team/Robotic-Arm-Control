#!/usr/bin/env python3

import argparse
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


class FkJointStateToArmBridge(Node):
    def __init__(
        self,
        publish_trajectory: bool,
        publish_joint_states: bool,
        publish_rate_hz: float,
        soft_limit_margin_rad: float,
        max_step_rad: float,
        joint2_sign_mode: str,
    ):
        super().__init__('fk_joint_state_to_arm_bridge')
        self.publish_trajectory = publish_trajectory
        self.publish_joint_states = publish_joint_states
        self.publish_rate_hz = max(1.0, float(publish_rate_hz))
        self.soft_limit_margin_rad = max(0.0, float(soft_limit_margin_rad))
        self.max_step_rad = max(0.01, float(max_step_rad))
        self.joint2_sign_mode = joint2_sign_mode

        self.arm_joints = ["Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"]
        self.joint_limits = self._load_joint_limits_from_urdf()
        self.gui_name_to_arm = {
            "joint1": "Joint_1",
            "joint2": "Joint_2",
            "joint3": "Joint_3",
            "joint4": "Joint_4",
            "joint5": "Joint_5",
            "joint_1": "Joint_1",
            "joint_2": "Joint_2",
            "joint_3": "Joint_3",
            "joint_4": "Joint_4",
            "joint_5": "Joint_5",
            "Joint_1": "Joint_1",
            "Joint_2": "Joint_2",
            "Joint_3": "Joint_3",
            "Joint_4": "Joint_4",
            "Joint_5": "Joint_5",
        }

        self.traj_pub = None
        if self.publish_trajectory:
            self.traj_pub = self.create_publisher(
                JointTrajectory,
                '/arm_controller_controller/joint_trajectory',
                10,
            )

        self.joint_state_pub = None
        if self.publish_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.latest_positions = None
        self.has_new_command = False
        self.last_sent_positions = None
        self._hold_publish_ticks = 0
        self._hold_publish_interval_ticks = max(1, int(round(self.publish_rate_hz * 0.5)))
        self.joint2_sign = None

        self.create_subscription(JointState, '/fk_joint_states', self._on_fk_joint_state, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_latest)

        self.get_logger().info(
            'Bridge active: /fk_joint_states mapped to Joint_1..Joint_5 '
            f'(trajectory={self.publish_trajectory}, joint_states={self.publish_joint_states}, '
            f'rate={self.publish_rate_hz:.1f}Hz, ignoring joint6)'
        )
        self.get_logger().info(
            "Loaded limits: "
            + ", ".join(f"{j}[{self.joint_limits[j][0]:+.3f},{self.joint_limits[j][1]:+.3f}]" for j in self.arm_joints)
        )
        self.get_logger().info(
            f'Bridge safety: soft_limit_margin={self.soft_limit_margin_rad:.3f} rad, '
            f'max_step={self.max_step_rad:.3f} rad/update'
        )
        self.get_logger().info(f'Joint_2 sign mode: {self.joint2_sign_mode}')

    def _load_joint_limits_from_urdf(self):
        defaults = {
            "Joint_1": (-0.0170, 3.1590),
            "Joint_2": (-2.6350, 0.0170),
            "Joint_3": (-0.0170, 3.1590),
            "Joint_4": (-0.0170, 3.1590),
            "Joint_5": (-0.0170, 3.1590),
        }

        urdf_path = None
        if get_package_share_directory is not None:
            try:
                urdf_path = os.path.join(get_package_share_directory("legacy_pkg"), "urdf", "legacy.urdf")
            except Exception:
                urdf_path = None
        if urdf_path is None:
            urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "urdf", "legacy.urdf"))

        try:
            root = ET.parse(urdf_path).getroot()
            loaded = dict(defaults)
            for joint in root.findall("joint"):
                name = joint.get("name")
                if name not in loaded:
                    continue
                limit = joint.find("limit")
                if limit is None:
                    continue
                lo = float(limit.get("lower", str(loaded[name][0])))
                hi = float(limit.get("upper", str(loaded[name][1])))
                loaded[name] = (lo, hi)
            return loaded
        except Exception:
            return defaults

    def _normalize_value(self, joint_name: str, source_name: str | None, raw_value: float) -> float:
        value = float(raw_value)

        if abs(value) > (2.0 * math.pi + 0.5):
            value = math.radians(value)

        if joint_name == "Joint_2":
            if self.joint2_sign_mode == 'raw':
                value = value
            elif self.joint2_sign_mode == 'inverted':
                value = -value
            else:
                if value <= 0.0:
                    value = value
                    selected_sign = +1.0
                else:
                    value = -value
                    selected_sign = -1.0

                if self.joint2_sign is None:
                    self.joint2_sign = selected_sign
                    mode = "raw(negative) / inverted(positive)"
                    self.get_logger().info(f"Joint_2 auto mapping enabled: {mode}")

        lo, hi = self.joint_limits[joint_name]
        if (hi - lo) > 2.0 * self.soft_limit_margin_rad:
            lo += self.soft_limit_margin_rad
            hi -= self.soft_limit_margin_rad
        return max(lo, min(hi, value))

    def _on_fk_joint_state(self, msg: JointState):
        if not msg.position:
            return

        target = {}

        if msg.name and len(msg.name) == len(msg.position):
            for name, position in zip(msg.name, msg.position):
                mapped = self.gui_name_to_arm.get(name)
                if mapped in self.arm_joints:
                    target[mapped] = self._normalize_value(mapped, name, float(position))

        if len(target) < 5 and len(msg.position) >= 5:
            for idx, joint_name in enumerate(self.arm_joints):
                target.setdefault(
                    joint_name,
                    self._normalize_value(joint_name, None, float(msg.position[idx]))
                )

        if len(target) < 5:
            self.get_logger().warn('Received /fk_joint_states without enough joint data for joints 1-5')
            return

        self.latest_positions = [target[joint_name] for joint_name in self.arm_joints]
        self.has_new_command = True

    def _publish_latest(self):
        if self.latest_positions is None:
            return

        self._hold_publish_ticks += 1
        force_hold_publish = self._hold_publish_ticks >= self._hold_publish_interval_ticks
        if not self.has_new_command and not force_hold_publish:
            return

        positions = self.latest_positions
        target_positions = positions
        if self.last_sent_positions is not None:
            stepped_positions = []
            for current, target in zip(self.last_sent_positions, positions):
                delta = target - current
                if delta > self.max_step_rad:
                    stepped_positions.append(current + self.max_step_rad)
                elif delta < -self.max_step_rad:
                    stepped_positions.append(current - self.max_step_rad)
                else:
                    stepped_positions.append(target)
            positions = stepped_positions

        if self.last_sent_positions is not None:
            max_delta = max(abs(a - b) for a, b in zip(positions, self.last_sent_positions))
            if max_delta < 1e-4 and not force_hold_publish:
                self.has_new_command = False
                return

        if self.publish_joint_states and self.joint_state_pub is not None:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.arm_joints
            js.position = positions
            self.joint_state_pub.publish(js)

        if self.publish_trajectory and self.traj_pub is not None:
            traj = JointTrajectory()
            traj.joint_names = self.arm_joints
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 150_000_000
            traj.points.append(point)
            self.traj_pub.publish(traj)

        self.last_sent_positions = list(positions)
        if max(abs(a - b) for a, b in zip(positions, target_positions)) > 1e-4:
            self.has_new_command = True
        else:
            self.has_new_command = False
        self._hold_publish_ticks = 0


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--publish-trajectory', action='store_true',
                        help='Publish mapped arm command to /arm_controller_controller/joint_trajectory')
    parser.add_argument('--publish-joint-states', action='store_true',
                        help='Publish mapped JointState to /joint_states')
    parser.add_argument('--publish-rate-hz', type=float, default=20.0,
                        help='Max output publish rate in Hz (default: 20)')
    parser.add_argument('--soft-limit-margin-rad', type=float, default=0.03,
                        help='Safety margin from URDF joint limits in radians (default: 0.03)')
    parser.add_argument('--max-step-rad', type=float, default=0.18,
                        help='Max per-update command step in radians (default: 0.18)')
    parser.add_argument('--joint2-sign-mode', choices=['inverted', 'raw', 'auto'], default='auto',
                        help='Joint_2 GUI mapping mode (default: auto)')
    known_args, ros_args = parser.parse_known_args()

    if not known_args.publish_trajectory and not known_args.publish_joint_states:
        known_args.publish_trajectory = True

    rclpy.init(args=ros_args if ros_args else args)
    node = FkJointStateToArmBridge(
        publish_trajectory=known_args.publish_trajectory,
        publish_joint_states=known_args.publish_joint_states,
        publish_rate_hz=known_args.publish_rate_hz,
        soft_limit_margin_rad=known_args.soft_limit_margin_rad,
        max_step_rad=known_args.max_step_rad,
        joint2_sign_mode=known_args.joint2_sign_mode,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

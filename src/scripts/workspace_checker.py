#!/usr/bin/env python3
"""
workspace_checker.py — Reports how far the EE can move from its current position
----------------------------------------------------------------------------------
Samples thousands of valid joint configurations using your actual URDF joint limits,
computes FK for each via the /compute_fk service, then reports:
  - Total reachable workspace bounds (absolute XYZ min/max)
  - How far the EE can move from its CURRENT position in each direction
  - Current EE position

Run while the robot is launched (move_group must be up).

Usage:
  python3 workspace_checker.py
  python3 workspace_checker.py --samples 5000   (more samples = more accurate)
"""

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import numpy as np
import argparse, math

# =====================================================================
# Your actual URDF joint limits
# =====================================================================
JOINT_LIMITS = {
    "Joint_1": (-1.5808,  1.5808),
    "Joint_2": (-2.4535,  0.01),
    "Joint_3": (-0.1,     3.1416),
    "Joint_4": (-1.5808,  1.5808),
    "Joint_5": (-1.5808,  1.5808),
}
JOINT_NAMES  = list(JOINT_LIMITS.keys())
EE_LINK      = "Link_5"
BASE_FRAME   = "base_link"
WORLD_FRAME  = "world"
# =====================================================================


class WorkspaceChecker(Node):
    def __init__(self):
        super().__init__("workspace_checker")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.current_joints: dict[str, float] = {}
        self.js_received = False
        self.create_subscription(
            JointState, "joint_states", self._js_cb, 10
        )

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos
        self.js_received = True

    def wait_for_joints(self, timeout=5.0):
        import time
        t0 = time.time()
        while not self.js_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                return False
        return True

    def compute_fk_batch(self, configs: list[list[float]]) -> list[tuple[float,float,float]]:
        """Compute FK for a batch of joint configs. Returns list of (x,y,z) in world frame."""
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_fk service not available")
            return []

        positions = []

        for config in configs:

            req = GetPositionFK.Request()
            req.header.frame_id = WORLD_FRAME
            req.fk_link_names = [EE_LINK]

            rs = RobotState()
            rs.joint_state.name = JOINT_NAMES
            rs.joint_state.position = config
            req.robot_state = rs

            future = self.fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.done() and future.result() is not None:
                result = future.result()
                if result.error_code.val == 1 and result.pose_stamped:
                    p = result.pose_stamped[0].pose.position
                    positions.append((p.x, p.y, p.z))

        return positions

    def get_current_ee_pos(self):
        try:
            t = self.tf_buffer.lookup_transform(
                WORLD_FRAME, EE_LINK,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            p = t.transform.translation
            return (p.x, p.y, p.z)
        except Exception as e:
            self.get_logger().error(f"TF failed: {e}")
            return None

    def run(self, n_samples: int, lock=False, minimal=False):
        print("\n" + "═"*60)
        print("  ROAR Arm Workspace Checker")
        print("═"*60)

        # Wait for joint states
        print("\n  Waiting for joint states...")
        if not self.wait_for_joints():
            print("  ERROR: No joint states received. Is the robot running?")
            return

        # Current EE position
        print("  Getting current EE position...")
        rclpy.spin_once(self, timeout_sec=1.0)
        current_pos = self.get_current_ee_pos()

        if current_pos:
            cx, cy, cz = current_pos
            print(f"\n  Current EE position (world frame):")
            print(f"    x = {cx:.4f} m")
            print(f"    y = {cy:.4f} m")
            print(f"    z = {cz:.4f} m")
        else:
            cx = cy = cz = 0.0
            print("  WARNING: Could not get current EE position")

        print("\n" + "─"*60)
        print("  KEY RESULT 1: ROTATION HEADROOM")
        print("─"*60)
        print(f"  Joint state + remaining rotation:")
        for jn in JOINT_NAMES:
            v = self.current_joints.get(jn, 0.0)
            lo, hi = JOINT_LIMITS[jn]
            pct_lo = 100 * (v - lo) / (hi - lo) if hi != lo else 0
            room_neg = math.degrees(v - lo)
            room_pos = math.degrees(hi - v)
            print(f"    {jn}: {v:+.4f} rad ({math.degrees(v):+.1f}°)  "
                  f"[{math.degrees(lo):.0f}° to {math.degrees(hi):.0f}°]  "
                  f"at {pct_lo:.0f}% | remaining: -{room_neg:.1f}° / +{room_pos:.1f}°")

        print("\n  Teleop rotation remaining (your controls):")
        for axis, joint_name in [("rz", "Joint_1"), ("ry", "Joint_2"), ("rx", "Joint_3")]:
            v = self.current_joints.get(joint_name, 0.0)
            lo, hi = JOINT_LIMITS[joint_name]
            print(f"    {axis}: {joint_name} -> -{math.degrees(v-lo):.1f}° / +{math.degrees(hi-v):.1f}°")

        # Sample random configurations
        print(f"\n  Sampling {n_samples} random joint configurations...")
        np.random.seed(42)
        configs = []
        current_cfg = [self.current_joints.get(jn, 0.0) for jn in JOINT_NAMES]
        configs.append(current_cfg)
        if lock:
            print("  Orientation lock enabled: Joint_4 and Joint_5 fixed to current values.")
            j4_val = self.current_joints.get("Joint_4", 0.0)
            j5_val = self.current_joints.get("Joint_5", 0.0)
            for _ in range(max(0, n_samples - 1)):
                cfg = [
                    np.random.uniform(JOINT_LIMITS["Joint_1"][0], JOINT_LIMITS["Joint_1"][1]),
                    np.random.uniform(JOINT_LIMITS["Joint_2"][0], JOINT_LIMITS["Joint_2"][1]),
                    np.random.uniform(JOINT_LIMITS["Joint_3"][0], JOINT_LIMITS["Joint_3"][1]),
                    j4_val,
                    j5_val
                ]
                configs.append(cfg)
        else:
            for _ in range(max(0, n_samples - 1)):
                cfg = [
                    np.random.uniform(lo, hi)
                    for lo, hi in JOINT_LIMITS.values()
                ]
                configs.append(cfg)

        print(f"  Running FK for {len(configs)} configurations...")

        # Compute FK
        positions = self.compute_fk_batch(configs)

        if not positions:
            print("  ERROR: FK computation returned no results.")
            return

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        print(f"\n  Successfully computed {len(positions)} FK samples")

        # Overall workspace bounds (reference)
        if not minimal:
            print("\n" + "─"*60)
            print("  REFERENCE: TOTAL REACHABLE WORKSPACE (world frame):")
            print("─"*60)
            print(f"    X: {min(xs):.4f} to {max(xs):.4f} m  "
                  f"(span: {max(xs)-min(xs):.4f} m)")
            print(f"    Y: {min(ys):.4f} to {max(ys):.4f} m  "
                  f"(span: {max(ys)-min(ys):.4f} m)")
            print(f"    Z: {min(zs):.4f} to {max(zs):.4f} m  "
                  f"(span: {max(zs)-min(zs):.4f} m)")


        # Reachable translation from current pose
        if current_pos:
            print("\n" + "─"*60)
            mode_label = "(J4/J5 LOCKED)" if lock else "(ALL JOINTS FREE)"
            print(f"  KEY RESULT 2: REACHABLE TRANSLATION FROM CURRENT EE POSITION {mode_label} "
                  f"({cx:.3f}, {cy:.3f}, {cz:.3f}):")
            print("─"*60)
            dxs = [x - cx for x, y, z in positions]
            dys = [y - cy for x, y, z in positions]
            dzs = [z - cz for x, y, z in positions]
            print(f"    +X (forward): up to {max(dxs)*100:.1f} cm")
            print(f"    -X (back):    up to {-min(dxs)*100:.1f} cm")
            print(f"    +Y (left):    up to {max(dys)*100:.1f} cm")
            print(f"    -Y (right):   up to {-min(dys)*100:.1f} cm")
            print(f"    +Z (up):      up to {max(dzs)*100:.1f} cm")
            print(f"    -Z (down):    up to {-min(dzs)*100:.1f} cm")

        print("═"*60 + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=2000,
                        help="Number of FK samples (default: 2000)")
    parser.add_argument("--lock", action="store_true",
                        help="Lock Joint_4 and Joint_5 to current values (orientation lock)")
    parser.add_argument("--minimal", action="store_true",
                        help="Show only key results (rotation headroom + reachable translation)")
    args = parser.parse_args()

    rclpy.init()
    node = WorkspaceChecker()
    node.run(args.samples, lock=args.lock, minimal=args.minimal)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
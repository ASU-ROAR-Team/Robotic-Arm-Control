#!/usr/bin/env python3
"""
workspace_viz.py — Visualize arm workspace as a point cloud in RViz
---------------------------------------------------------------------
Samples random joint configurations within your actual URDF limits,
computes FK for each, and publishes the reachable positions as a
MarkerArray in RViz. Also shows current EE position as a large sphere.

SETUP IN RVIZ:
  1. Run this script
  2. In RViz → Add → By topic → /workspace_cloud → MarkerArray
  3. Also add /workspace_ee_pos → MarkerArray for current EE marker
  4. Set Fixed Frame to "world"

Run while the robot is launched (move_group must be up).

Usage:
  python3 workspace_viz.py
  python3 workspace_viz.py --samples 3000 --point-size 0.01
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
import argparse, math, sys, time

# =====================================================================
# Your actual URDF joint limits
# =====================================================================
JOINT_LIMITS = {
    "Joint_1": (-0.0170,  3.1590),
    "Joint_2": (-2.6350,  0.0170),
    "Joint_3": (-0.0170,  3.1590),
    "Joint_4": (-0.0170,  3.1590),
    "Joint_5": (-0.0170,  3.1590),
}
JOINT_NAMES = list(JOINT_LIMITS.keys())
EE_LINK     = "Link_5"
WORLD_FRAME = "world"
BASE_FRAME  = "base_link"
# =====================================================================


def height_color(z: float, z_min: float, z_max: float) -> ColorRGBA:
    """Color workspace points by height: blue (low) → green → red (high)."""
    t = (z - z_min) / (z_max - z_min + 1e-9)
    c = ColorRGBA()
    if t < 0.5:
        c.r = 0.0
        c.g = 2.0 * t
        c.b = 1.0 - 2.0 * t
    else:
        c.r = 2.0 * (t - 0.5)
        c.g = 1.0 - 2.0 * (t - 0.5)
        c.b = 0.0
    c.a = 0.6
    return c


class WorkspaceViz(Node):
    def __init__(self, n_samples: int, point_size: float):
        super().__init__("workspace_viz")
        self.n_samples  = n_samples
        self.point_size = point_size

        self.cloud_pub  = self.create_publisher(MarkerArray, "/workspace_cloud", 1)
        self.ee_pub     = self.create_publisher(MarkerArray, "/workspace_ee_pos", 1)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.fk_client   = self.create_client(GetPositionFK, "/compute_fk")

        self.current_joints: dict[str, float] = {}
        self.js_received = False
        self.create_subscription(JointState, "joint_states", self._js_cb, 10)

        # Republish cloud every 2 seconds so RViz doesn't lose it
        self.cached_cloud: MarkerArray | None = None
        self.create_timer(2.0, self._republish)

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current_joints[n] = p
        self.js_received = True

    def _republish(self):
        if self.cached_cloud:
            self.cloud_pub.publish(self.cached_cloud)
        self._update_ee_marker()

    def _update_ee_marker(self):
        try:
            t = self.tf_buffer.lookup_transform(
                WORLD_FRAME, EE_LINK,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            p = t.transform.translation
        except Exception:
            return

        m = Marker()
        m.header.frame_id = WORLD_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "ee_current"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = p.x
        m.pose.position.y = p.y
        m.pose.position.z = p.z
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.04  # 4cm sphere
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        m.lifetime = rclpy.duration.Duration(seconds=3).to_msg()

        ma = MarkerArray()
        ma.markers.append(m)
        self.ee_pub.publish(ma)

    def compute_fk_batch(self, configs):
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_fk not available")
            return []

        positions = []
        total = len(configs)
        report_every = max(1, total // 20)

        for i, cfg in enumerate(configs):
            if i % report_every == 0:
                sys.stdout.write(f"\r  FK sampling: {i}/{total} ({100*i//total}%)")
                sys.stdout.flush()

            req = GetPositionFK.Request()
            req.header.frame_id = WORLD_FRAME
            req.fk_link_names = [EE_LINK]
            rs = RobotState()
            rs.joint_state.name = JOINT_NAMES
            rs.joint_state.position = cfg
            req.robot_state = rs

            fut = self.fk_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=0.5)
            if fut.done() and fut.result() and fut.result().error_code.val == 1:
                if fut.result().pose_stamped:
                    p = fut.result().pose_stamped[0].pose.position
                    positions.append((p.x, p.y, p.z))

        sys.stdout.write(f"\r  FK sampling: {total}/{total} (100%)\n")
        return positions

    def build_cloud_marker(self, positions):
        """
        Use SPHERE_LIST for efficiency — one marker with thousands of points.
        Color by height (blue=low, green=mid, red=high).
        """
        if not positions:
            return None

        zs = [p[2] for p in positions]
        z_min, z_max = min(zs), max(zs)

        m = Marker()
        m.header.frame_id = WORLD_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = self.point_size
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 0 = forever

        for x, y, z in positions:
            pt = Point()
            pt.x, pt.y, pt.z = x, y, z
            m.points.append(pt)
            m.colors.append(height_color(z, z_min, z_max))

        return m

    def build_bounds_markers(self, positions):
        """Add axis-aligned bounding box lines around the workspace."""
        if not positions:
            return []
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        z0, z1 = min(zs), max(zs)

        # 12 edges of bounding box
        corners = [
            (x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
            (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1),
        ]
        edges = [
            (0,1),(1,2),(2,3),(3,0),   # bottom
            (4,5),(5,6),(6,7),(7,4),   # top
            (0,4),(1,5),(2,6),(3,7),   # verticals
        ]

        m = Marker()
        m.header.frame_id = WORLD_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace_bounds"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.005  # 5mm line width
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.8
        m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        for i, j in edges:
            for idx in (i, j):
                pt = Point()
                pt.x, pt.y, pt.z = corners[idx]
                m.points.append(pt)

        return [m]

    def run(self):
        print("\n" + "═"*60)
        print("  ROVER Arm Workspace Visualizer")
        print("═"*60)
        print(f"  Samples: {self.n_samples}")
        print(f"  Point size: {self.point_size*100:.1f} cm")
        print(f"  EE link: {EE_LINK}")
        print()
        print("  RVIZ SETUP:")
        print("    1. Add > By Topic > /workspace_cloud > MarkerArray")
        print("    2. Add > By Topic > /workspace_ee_pos > MarkerArray")
        print("    3. Fixed Frame = 'world'")
        print("    Colors: blue=low, green=mid, red=high altitude")
        print("    Yellow sphere = current EE position (updates live)")
        print()

        # Wait for joint states
        print("  Waiting for joint states...", end=" ")
        t0 = time.time()
        while not self.js_received and time.time() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        print("OK" if self.js_received else "TIMEOUT (continuing anyway)")

        # Sample FK
        np.random.seed(0)
        configs = [
            [np.random.uniform(lo, hi) for lo, hi in JOINT_LIMITS.values()]
            for _ in range(self.n_samples)
        ]
        positions = self.compute_fk_batch(configs)

        if not positions:
            print("  ERROR: No FK results. Is move_group running?")
            return

        print(f"  Got {len(positions)} valid positions")

        # Print summary
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        print(f"\n  Workspace bounds (world frame):")
        print(f"    X: {min(xs):.3f} to {max(xs):.3f} m  (span {(max(xs)-min(xs))*100:.1f} cm)")
        print(f"    Y: {min(ys):.3f} to {max(ys):.3f} m  (span {(max(ys)-min(ys))*100:.1f} cm)")
        print(f"    Z: {min(zs):.3f} to {max(zs):.3f} m  (span {(max(zs)-min(zs))*100:.1f} cm)")

        dists = [math.sqrt(x**2+y**2+z**2) for x,y,z in positions]
        print(f"\n  Radial reach from base:")
        print(f"    Min: {min(dists)*100:.1f} cm")
        print(f"    Max: {max(dists)*100:.1f} cm")

        # Build and publish markers
        cloud_m = self.build_cloud_marker(positions)
        bound_m = self.build_bounds_markers(positions)

        ma = MarkerArray()
        if cloud_m:
            ma.markers.append(cloud_m)
        ma.markers.extend(bound_m)
        self.cached_cloud = ma
        self.cloud_pub.publish(ma)
        self._update_ee_marker()

        print(f"\n  Published to RViz. Cloud republishes every 2s.")
        print(f"  Yellow sphere tracks current EE position live.")
        print(f"  Ctrl+C to stop.\n")

        rclpy.spin(self)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples",    type=int,   default=2000,
                        help="Number of FK samples (default 2000, more=better)")
    parser.add_argument("--point-size", type=float, default=0.008,
                        help="Sphere radius in metres (default 0.008 = 8mm)")
    args = parser.parse_args()

    rclpy.init()
    node = WorkspaceViz(args.samples, args.point_size)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
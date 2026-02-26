#!/usr/bin/env python3
# ws_ros2_bridge.py

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
import json
import asyncio
import threading
import base64
import websockets

PORT = 8080


class WSROS2Bridge(Node):
    def __init__(self):
        super().__init__('ws_ros2_bridge')

        # ---------------- ROS2 Publishers ----------------
        self.joint_pub    = self.create_publisher(JointState,        '/fk_joint_states',               10)
        self.pose_pub     = self.create_publisher(Float64MultiArray,  '/ik_target_pose',                10)
        self.mission_pub  = self.create_publisher(String,             '/mission_cmd',                   10)
        self.drilling_pub = self.create_publisher(String,             '/drilling/command_to_actuators',  10)
        self.cmd_vel_pub  = self.create_publisher(Twist,              '/cmd_vel',                       10)

        # ---------------- ROS2 Subscribers ----------------
        self.create_subscription(String,          '/rover_status',                    self.rover_status_cb,    10)
        self.create_subscription(String,          '/node_status',                     self.node_status_cb,     10)
        self.create_subscription(String,          '/drilling/feedback',               self.drilling_status_cb, 10)
        self.create_subscription(String,          '/drilling_fsm_state',              self.drilling_fsm_cb,    10)

        # Cameras
        self.create_subscription(
            CompressedImage, '/logitech_1/image_raw/compressed',
            self.logitech_camera_cb, 10)
        self.create_subscription(
            CompressedImage,
            '/zed2i/zed_node/depth/depth_registered/color_mapped_image/compressed_for_web',
            self.zed_camera_cb, 10)

        # Costmap / navigation topics
        self.create_subscription(Odometry, '/filtered_state',    self.robot_pose_cb,     10)
        self.create_subscription(Path,     '/Path',              self.global_path_cb,    10)
        self.create_subscription(Path,     '/traversed_path',    self.traversed_path_cb, 10)
        self.create_subscription(Marker,   '/obstacles',         self.obstacle_cb,       10)

        # ---------------- Internal ----------------
        self.ws_clients   = set()
        self.rover_status = {
            "rover_state": "IDLE",
            "active_mission": "",
            "supervisor_message": "",
            "node_statuses": []
        }

        # asyncio event loop in a dedicated thread
        self.loop      = asyncio.new_event_loop()
        self.ws_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.ws_thread.start()

    # ── asyncio loop thread ──────────────────────────────────────────────────

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._ws_server())

    async def _ws_server(self):
        async with websockets.serve(self._handler, "0.0.0.0", PORT):
            self.get_logger().info(f"WebSocket server running on ws://0.0.0.0:{PORT}")
            await asyncio.Future()

    # ── WebSocket handler ────────────────────────────────────────────────────

    async def _handler(self, websocket):
        self.ws_clients.add(websocket)
        self.get_logger().info(f"Client connected: {websocket.remote_address}")
        try:
            async for message in websocket:
                await self._handle_message(message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)
            self.get_logger().info(f"Client disconnected: {websocket.remote_address}")

    async def _handle_message(self, message):
        try:
            msg      = json.loads(message)
            msg_type = msg.get("type")

            if msg_type == "joint_cmd":
                mode = msg.get("mode")
                data = msg.get("data", [])
                if mode == "FK" and len(data) == 6:
                    joint_msg          = JointState()
                    joint_msg.name     = [f'joint{i+1}' for i in range(6)]
                    joint_msg.position = [float(x) for x in data]
                    self.joint_pub.publish(joint_msg)
                elif mode == "IK":
                    pose_msg      = Float64MultiArray()
                    pose_msg.data = [float(x) for x in data]
                    self.pose_pub.publish(pose_msg)

            elif msg_type == "mission_cmd":
                out      = String()
                out.data = json.dumps({"command": msg.get("command", ""), "mission": msg.get("mission", "")})
                self.mission_pub.publish(out)

            elif msg_type == "drilling_cmd":
                out      = String()
                out.data = json.dumps(msg.get("data", {}))
                self.drilling_pub.publish(out)

            elif msg_type == "cmd_vel":
                data    = msg.get("data", {})
                linear  = data.get("linear",  {})
                angular = data.get("angular", {})
                twist           = Twist()
                twist.linear.x  = float(linear.get("x",  0.0))
                twist.linear.y  = float(linear.get("y",  0.0))
                twist.linear.z  = float(linear.get("z",  0.0))
                twist.angular.x = float(angular.get("x", 0.0))
                twist.angular.y = float(angular.get("y", 0.0))
                twist.angular.z = float(angular.get("z", 0.0))
                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Failed to handle WS message: {e}")

    # ── broadcast ────────────────────────────────────────────────────────────

    def broadcast(self, payload: str):
        if not self.ws_clients:
            return
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self.loop)

    async def _broadcast(self, payload: str):
        dead = set()
        for client in self.ws_clients:
            try:
                await client.send(payload)
            except Exception:
                dead.add(client)
        self.ws_clients -= dead

    # ── ROS2 callbacks ───────────────────────────────────────────────────────

    def rover_status_cb(self, msg):
        try:    self.rover_status = json.loads(msg.data)
        except Exception as e: self.get_logger().warn(f"Failed to parse rover_status: {e}")
        self.broadcast(json.dumps({"type": "rover_status", "data": json.dumps(self.rover_status)}))

    def node_status_cb(self, msg):
        self.broadcast(json.dumps({"type": "node_status", "data": msg.data}))

    def drilling_status_cb(self, msg):
        self.broadcast(json.dumps({"type": "drilling_status", "data": msg.data}))

    def drilling_fsm_cb(self, msg):
        self.broadcast(json.dumps({"type": "drilling_fsm_state", "data": json.dumps({"data": msg.data})}))

    def logitech_camera_cb(self, msg):
        if not self.ws_clients: return
        b64 = base64.b64encode(bytes(msg.data)).decode('utf-8')
        self.broadcast(json.dumps({"type": "camera_frame", "data": json.dumps({"data": b64})}))

    def zed_camera_cb(self, msg):
        if not self.ws_clients: return
        b64 = base64.b64encode(bytes(msg.data)).decode('utf-8')
        self.broadcast(json.dumps({"type": "zed_frame", "data": json.dumps({"data": b64})}))

    def robot_pose_cb(self, msg):
        # nav_msgs/Odometry → JSON matching the shape the plugin expects
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        payload = {
            "pose": {
                "pose": {
                    "position":    {"x": pos.x, "y": pos.y, "z": pos.z},
                    "orientation": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w}
                }
            }
        }
        self.broadcast(json.dumps({"type": "robot_pose", "data": json.dumps(payload)}))

    def global_path_cb(self, msg):
        # nav_msgs/Path → JSON
        poses = [
            {"pose": {"position": {"x": p.pose.position.x, "y": p.pose.position.y, "z": p.pose.position.z}}}
            for p in msg.poses
        ]
        self.broadcast(json.dumps({"type": "global_path", "data": json.dumps({"poses": poses})}))

    def traversed_path_cb(self, msg):
        # nav_msgs/Path → JSON
        poses = [
            {"pose": {"position": {"x": p.pose.position.x, "y": p.pose.position.y, "z": p.pose.position.z}}}
            for p in msg.poses
        ]
        self.broadcast(json.dumps({"type": "traversed_path", "data": json.dumps({"poses": poses})}))

    def obstacle_cb(self, msg):
        # visualization_msgs/Marker → JSON
        # replaces roar_msgs/Obstacle:
        #   msg.id           (was msg.id.data      — std_msgs/Int8)
        #   msg.pose         (was msg.position.pose — geometry_msgs/PoseStamped)
        #   msg.scale.x / 2  (was msg.radius.data  — std_msgs/Float32, scale = diameter)
        payload = {
            "id": msg.id,
            "pose": {
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z
                }
            },
            "scale": {"x": msg.scale.x, "y": msg.scale.y, "z": msg.scale.z}
        }
        self.broadcast(json.dumps({"type": "obstacle", "data": json.dumps(payload)}))


# ── entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WSROS2Bridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
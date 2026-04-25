#!/usr/bin/env python3
"""
read_ee_direction.py

Simple ROS2 helper to read the transform from `ee_ref` -> `link_6` and
print the end-effector's forward (+X) direction expressed in the `ee_ref`
frame. Run while the simulation is running.
"""
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

LINK_NAME = "link_6"
REF_FRAME = "ee_ref"


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


def rotate_vector(q, v):
    # rotate vector v by quaternion q (x,y,z,w)
    x, y, z, w = q
    vx, vy, vz = v
    ix = w * vx + y * vz - z * vy
    iy = w * vy + z * vx - x * vz
    iz = w * vz + x * vy - y * vx
    iw = -x * vx - y * vy - z * vz
    rx = ix * w + iw * -x + iy * -z - iz * -y
    ry = iy * w + iw * -y + iz * -x - ix * -z
    rz = iz * w + iw * -z + ix * -y - iy * -x
    return (rx, ry, rz)


def normalize(v):
    nx, ny, nz = v
    n = math.sqrt(nx * nx + ny * ny + nz * nz)
    if n == 0:
        return (0.0, 0.0, 0.0)
    return (nx / n, ny / n, nz / n)


def main():
    rclpy.init()
    node = rclpy.create_node('ee_direction_reader')
    tf_buffer = Buffer()
    _ = TransformListener(tf_buffer, node)

    # give the TransformListener a short time to populate the buffer
    node.get_logger().info('EE direction reader started. Warming up TF listener...')
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.get_logger().info('Warm-up complete. Waiting for TF...')

    try:
        while rclpy.ok():
            try:
                # Try multiple reference frames: prefer ee_ref, fall back to world or base_link
                frames_to_try = [REF_FRAME, 'world', 'base_link']
                tf = None
                used_parent = None
                for parent in frames_to_try:
                    try:
                        if tf_buffer.can_transform(parent, LINK_NAME, Time(), timeout=Duration(seconds=0.5)):
                            tf = tf_buffer.lookup_transform(parent, LINK_NAME, Time(), timeout=Duration(seconds=0.5))
                            used_parent = parent
                            break
                    except Exception:
                        continue

                if tf is None:
                    # wait and retry
                    rclpy.spin_once(node, timeout_sec=0.1)
                    try:
                        frames = tf_buffer.all_frames_as_yaml()
                        node.get_logger().info(f'No suitable parent frame found. Available frames:\n{frames}')
                    except Exception:
                        node.get_logger().info('No suitable parent frame found and unable to list frames.')
                    time.sleep(0.2)
                    continue

                q = (
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                )
                # robot convention: in home, -X is forward, +X is backwards,
                # +Y is right, -Y is left (so local forward = -X)
                local_forward = (-1.0, 0.0, 0.0)
                # forward expressed in REF_FRAME
                fwd = rotate_vector(q, local_forward)
                fwd_u = normalize(fwd)

                # determine nearest semantic direction in REF_FRAME using robot convention
                axes = [('+X', (1, 0, 0), 'Backward'), ('-X', (-1, 0, 0), 'Forward'), ('+Y', (0, 1, 0), 'Right'), ('-Y', (0, -1, 0), 'Left'), ('+Z', (0, 0, 1), 'Up'), ('-Z', (0, 0, -1), 'Down')]
                best = None
                best_score = -1.0
                best_semantic = None
                for name, axis, semantic in axes:
                    score = fwd_u[0] * axis[0] + fwd_u[1] * axis[1] + fwd_u[2] * axis[2]
                    if score > best_score:
                        best_score = score
                        best = name
                        best_semantic = semantic

                roll, pitch, yaw = euler_from_quat(*q)
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)

                node.get_logger().info(
                    f'Forward (in {used_parent}): [{fwd_u[0]:+.3f}, {fwd_u[1]:+.3f}, {fwd_u[2]:+.3f}] -> mostly {best_semantic} '
                    f'| RPY (deg): {roll_deg:+.1f}, {pitch_deg:+.1f}, {yaw_deg:+.1f}'
                )
            except Exception as exc:
                # On TF lookup failure, try to print available frames for debugging
                try:
                    frames = tf_buffer.all_frames_as_yaml()
                    node.get_logger().warn(f'TF lookup failed: {exc} -- Available frames:\n{frames}')
                except Exception:
                    node.get_logger().warn(f'TF lookup failed: {exc}')
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

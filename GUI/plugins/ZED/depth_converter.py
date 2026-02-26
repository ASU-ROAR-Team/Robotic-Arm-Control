#!/usr/bin/env python3
# depthconverter.py — ROS2 version

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# --- Fixed depth range (metres) — adjust to suit your scene ---
DEPTH_MIN = 0.5
DEPTH_MAX = 10.0
# --------------------------------------------------------------


class DepthImageConverter(Node):
    def __init__(self):
        super().__init__('depth_image_converter')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/zed2i/zed_node/depth/depth_registered',
            self.callback,
            1          # queue_size=1 — only keep latest frame
        )

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            '/zed2i/zed_node/depth/depth_registered/color_mapped_image/compressed_for_web',
            1
        )

        self.get_logger().info('Depth Image Converter Node started (ROS2).')
        self.get_logger().info('Subscribing to /zed2i/zed_node/depth/depth_registered (32FC1)')
        self.get_logger().info('Publishing  to /zed2i/.../compressed_for_web (JPEG)')

    def callback(self, data: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if cv_image is None:
            self.get_logger().warn('Received empty image.')
            return

        # Replace NaN / Inf and clamp to fixed range
        processed = np.copy(cv_image)
        processed[np.isnan(processed)] = 0.0
        processed[np.isinf(processed)] = 0.0
        processed = np.clip(processed, DEPTH_MIN, DEPTH_MAX)

        # Normalise to 0-255
        if DEPTH_MAX == DEPTH_MIN:
            normalised = np.zeros_like(processed, dtype=np.uint8)
        else:
            normalised = ((processed - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 255).astype(np.uint8)

        # Apply colour map
        colour_mapped = cv2.applyColorMap(normalised, cv2.COLORMAP_JET)

        # Compress to JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        try:
            result, encoded = cv2.imencode('.jpg', colour_mapped, encode_param)
            if not result:
                self.get_logger().error('Failed to encode image to JPEG.')
                return

            msg          = CompressedImage()
            msg.header   = data.header   # preserve original timestamp / frame_id
            msg.format   = 'jpeg'
            msg.data     = np.array(encoded).tobytes()
            self.compressed_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Compression / publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthImageConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
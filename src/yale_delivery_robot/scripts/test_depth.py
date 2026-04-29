#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


INPUT_TOPIC = "/camera/depth/image_raw"
OUTPUT_TOPIC = "/camera/depth_debug_image"

MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 5.0


class DepthDebugNode(Node):
    def __init__(self):
        super().__init__("depth_debug_node")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            INPUT_TOPIC,
            self.callback,
            10
        )
        self.pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        self.get_logger().info(f"Subscribed to {INPUT_TOPIC}")
        self.get_logger().info(f"Publishing display image to {OUTPUT_TOPIC}")

    def callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"Depth convert failed: {e}")
            return

        depth = np.array(depth, dtype=np.float32)

        # If depth is in millimeters, convert to meters when needed
        if msg.encoding == "16UC1":
            depth = depth / 1000.0

        valid = np.isfinite(depth) & (depth > MIN_DEPTH_M) & (depth < MAX_DEPTH_M)

        display = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth, MIN_DEPTH_M, MAX_DEPTH_M)
            norm = (clipped - MIN_DEPTH_M) / (MAX_DEPTH_M - MIN_DEPTH_M)
            # invert so nearer = brighter
            display = (255 * (1.0 - norm)).astype(np.uint8)
            display[~valid] = 0

        # Optional color map for easier viewing
        color = cv2.applyColorMap(display, cv2.COLORMAP_JET)

        try:
            out = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
            out.header = msg.header
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"Publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
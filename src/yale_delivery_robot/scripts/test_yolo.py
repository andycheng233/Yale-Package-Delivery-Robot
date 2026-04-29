#!/usr/bin/env python3

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String


YOLO_MODEL = "yolov8n.pt"
YOLO_CONF = 0.4
YOLO_DEVICE = "cpu"   # use "0" if you want GPU
INPUT_TOPIC = "/camera/image_raw/compressed"
OUTPUT_TOPIC = "/yolo_debug_image"
STATUS_TOPIC = "/yolo_debug_status"


class YoloPersonDebugNode(Node):
    def __init__(self):
        super().__init__("test_yolo")

        self.bridge = CvBridge()
        self.last_inference_time = 0.0
        self.min_inference_period = 0.1  # 10 Hz max

        self.frame_count = 0
        self.last_log_time = time.monotonic()

        try:
            from ultralytics import YOLO
            self.model = YOLO(YOLO_MODEL)
            self.get_logger().info(f"Loaded YOLO model: {YOLO_MODEL} on {YOLO_DEVICE}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO: {e}")
            raise

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            INPUT_TOPIC,
            self.image_callback,
            qos,
        )

        self.debug_pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        self.get_logger().info(f"Subscribed to: {INPUT_TOPIC}")
        self.get_logger().info(f"Publishing debug image to: {OUTPUT_TOPIC}")

    def image_callback(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if now - self.last_inference_time < self.min_inference_period:
            return
        self.last_inference_time = now

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn("Failed to decode compressed image")
                return
        except Exception as e:
            self.get_logger().warn(f"Failed to decode compressed image: {e}")
            return
        
        annotated, num_people = self.run_yolo(frame)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.debug_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

        status_msg = String()
        status_msg.data = f"people_detected={num_people}"
        self.status_pub.publish(status_msg)

        self.frame_count += 1
        if now - self.last_log_time > 2.0:
            self.get_logger().info(
                f"Frames processed: {self.frame_count}, people in latest frame: {num_people}"
            )
            self.last_log_time = now

    def run_yolo(self, frame):
        results = self.model(
            frame,
            classes=[0],  # person only
            conf=YOLO_CONF,
            device=YOLO_DEVICE,
            verbose=False,
        )

        annotated = frame.copy()
        people_count = 0

        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0].item()) if box.cls is not None else -1
                if cls_id != 0:
                    continue

                conf = float(box.conf[0].item()) if box.conf is not None else 0.0
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]

                people_count += 1

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f"person {conf:.2f}",
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (0, 255, 0),
                    1,
                )

        cv2.putText(
            annotated,
            f"people: {people_count}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )

        return annotated, people_count


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = YoloPersonDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
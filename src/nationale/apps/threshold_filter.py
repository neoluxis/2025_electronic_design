import os


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np

import nlxpy
from nlxpy.ros2.imger import cv2ros, ros2cv


from nationale_interfaces.srv import ThresholdSetter


class ThresholdFilter(Node):
    def __init__(self):
        super().__init__("threshold_filter")
        self.get_logger().info("ThresholdFilter node has been initialized.")

        self.color_mode = "bgr"
        self.thresholds = [0, 255, 0, 255, 0, 255]

        self.image_topic = (
            self.declare_parameter("image_topic", "/image_mjpeg")
            .get_parameter_value()
            .string_value
        )

        self.setup_service = self.create_service(
            ThresholdSetter,
            "/threshold_filter/set_thresholds",
            self.set_thresholds_callback,
        )

        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.image_callback, 10
        )

        self.mask_pub = self.create_publisher(
            CompressedImage, "/threshold_filter/mask", 10
        )
        
        self.frame_count = 0
        self.fps_timer = self.create_timer(1.0, self.fps_timer_callback)

    def image_callback(self, msg: CompressedImage):
        img = ros2cv(msg)
        if img is None:
            self.get_logger().warn("Received an empty image.")
            return

        if self.color_mode == "bgr":
            pass
        elif self.color_mode == "hsv":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Apply thresholds
        lower_bound = (self.thresholds[0], self.thresholds[2], self.thresholds[4])
        upper_bound = (self.thresholds[1], self.thresholds[3], self.thresholds[5])
        mask = cv2.inRange(img, lower_bound, upper_bound)

        # Convert back to ROS message
        rmask = cv2ros(mask)
        mmsg = CompressedImage()
        mmsg.header = msg.header
        mmsg.format = "jpeg"
        mmsg.data = rmask
        self.mask_pub.publish(mmsg)
        self.frame_count += 1
        
    def fps_timer_callback(self):
        fps = self.frame_count
        self.get_logger().info(f"Processed {fps} frames in the last second.")
        self.frame_count = 0

    def set_thresholds_callback(self, request, response):
        self.color_mode = request.color_mode
        self.thresholds = request.thresholds
        self.get_logger().info(
            f"Thresholds set to {self.thresholds} with color mode {self.color_mode}"
        )
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ThresholdFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
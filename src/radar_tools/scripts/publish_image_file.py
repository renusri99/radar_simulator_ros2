#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

class ImageFilePublisher(Node):
    def __init__(self):
        super().__init__('publish_image_file')

        self.declare_parameter('file', '')

        filename = self.get_parameter('file').get_parameter_value().string_value

        self.br = CvBridge()
        radar_image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if radar_image is None:
            self.get_logger().error(f"Failed to load image from {filename}")
            rclpy.shutdown()
            return

        self.radar_image_msg = self.br.cv2_to_imgmsg(radar_image, encoding="passthrough")

        self.publisher = self.create_publisher(Image, '/Navtech/Polar', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz

    def timer_callback(self):
        self.radar_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.radar_image_msg.header.frame_id = "navtech"
        self.publisher.publish(self.radar_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageFilePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

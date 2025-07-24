#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from radar_tools.msg import ImageCompMetrics
import csv
import os

class ImageCompareWriter(Node):
    def __init__(self):
        super().__init__('compare_radar_images_writer')

        # Declare and get log file path
        self.declare_parameter('output_csv', 'image_compare.csv')
        self.csv_path = self.get_parameter('output_csv').get_parameter_value().string_value

        # Open CSV file and create writer
        self.outfile = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.outfile)

        self.writer.writerow([
            "stamp1", "stamp2", "ssi", "psnr", "nmi", "voi0", "voi1", "mis"
        ])

        # Subscribe to topic
        self.sub = self.create_subscription(
            ImageCompMetrics,
            'real_to_sim_gap',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info('Writing image comparison data to CSV')
        self.writer.writerow([
            msg.stamp1.sec + msg.stamp1.nanosec * 1e-9,
            msg.stamp2.sec + msg.stamp2.nanosec * 1e-9,
            msg.ssi,
            msg.psnr,
            msg.nmi,
            msg.voi[0],
            msg.voi[1],
            msg.mis
        ])

    def destroy_node(self):
        self.outfile.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompareWriter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

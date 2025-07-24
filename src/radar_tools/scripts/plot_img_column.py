#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as ticker

class PlotImgColumn(Node):
    def __init__(self):
        super().__init__('plot_img_column')

        self.declare_parameter('topic', '/radar/image')
        self.declare_parameter('col', 0)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.col = self.get_parameter('col').get_parameter_value().integer_value

        self.image = None
        self.br = CvBridge()
        self.sub = self.create_subscription(Image, self.topic, self.img_cb, 10)

        self.y_max = 150
        self.fig, self.ax = plt.subplots()
        self.ln, = self.ax.plot([], [])
        self.ax.set_ylim(0, self.y_max)
        self.ani = FuncAnimation(self.fig, self.update, interval=500, init_func=self.init, blit=True)

    def img_cb(self, msg):
        try:
            self.image = self.br.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def init(self):
        self.ax.set_ylim(0, self.y_max)
        return self.ln,

    def update(self, i):
        col_param = self.get_parameter('col').get_parameter_value().integer_value
        if col_param != self.col:
            self.col = col_param

        if self.image is not None:
            if self.col < self.image.shape[1]:
                self.ax.set_xlim(0, self.image.shape[0])
                ticks = ticker.FuncFormatter(lambda x, pos: '{0:g}'.format(x * 0.044))
                self.ax.xaxis.set_major_formatter(ticks)

                ydata = self.image[:, self.col].astype(float)
                xdata = np.arange(0, ydata.shape[0])
                self.ln.set_data(xdata, ydata)
        return self.ln,

    def plot(self):
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PlotImgColumn()
    try:
        node.plot()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

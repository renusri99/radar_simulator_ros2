#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from radar_tools.cfg import PlotImgColConfig

from cv_bridge import CvBridge
import cv2
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.ticker as ticker
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

class Plotter(Node):
    def __init__(self):
        super().__init__('plot3d_radar_image')

        self.declare_parameter('topic', '/Navtech/Polar')
        topic_name = self.get_parameter('topic').get_parameter_value().string_value

        self.col = 0
        self.br = CvBridge()
        self.image = None
        self.y_max = 1000
        self.z_max = 150

        self.fig, self.ax = plt.subplots(subplot_kw={"projection": "3d"})

        X = np.arange(0, 1000)
        Y = np.arange(0, 400)
        self.XX, self.YY = np.meshgrid(Y, X)
        ZZ = np.zeros((1000, 400), dtype=np.uint8)

        self.mem = np.array([self.XX, self.YY, ZZ]).T
        self.surf = self.ax.plot_surface(self.XX, self.YY, ZZ, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        self.ani = FuncAnimation(self.fig, self.update, interval=500, init_func=self.init, blit=True)

        self.create_subscription(Image, topic_name, self.img_cb, 10)

    def img_cb(self, msg):
        image_raw = self.br.imgmsg_to_cv2(msg)
        image = image_raw[:self.y_max]
        self.image = image

    def init(self):
        return self.surf,

    def update(self, i):
        if self.image is not None:
            self.ax.clear()
            self.surf = self.ax.plot_surface(self.XX, self.YY, self.image, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        return self.surf,

    def plot(self):
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = Plotter()
    node.plot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

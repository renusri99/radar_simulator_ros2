
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from radar_tools.msg import ImageCompMetrics

from cv_bridge import CvBridge
import cv2
import numpy as np

from skimage.metrics import structural_similarity, peak_signal_noise_ratio, normalized_mutual_information, variation_of_information
from sklearn.metrics import mutual_info_score

from message_filters import ApproximateTimeSynchronizer, Subscriber

class ImageComparer(Node):
    def __init__(self):
        super().__init__('compare_radar_images')

        self.br = CvBridge()

        self.declare_parameter('topic_in_1', '/Navtech/Polar')
        self.declare_parameter('topic_in_2', '/radar/image')
        self.declare_parameter('topic_out', '/real_to_sim_gap')
        self.declare_parameter('time_sync', 'virtual')

        self.topic_in_1 = self.get_parameter('topic_in_1').get_parameter_value().string_value
        self.topic_in_2 = self.get_parameter('topic_in_2').get_parameter_value().string_value
        topic_out = self.get_parameter('topic_out').get_parameter_value().string_value
        self.time_sync = self.get_parameter('time_sync').get_parameter_value().string_value

        self.pub = self.create_publisher(ImageCompMetrics, topic_out, 10)

        if self.time_sync == 'virtual':
            self.sub1 = Subscriber(self, Image, self.topic_in_1)
            self.sub2 = Subscriber(self, Image, self.topic_in_2)
            self.ts = ApproximateTimeSynchronizer([self.sub1, self.sub2], queue_size=10, slop=1.0)
            self.ts.registerCallback(self.img_cb)
        else:
            self.sub1 = self.create_subscription(Image, self.topic_in_1, self.img1_cb, 10)
            self.sub2 = self.create_subscription(Image, self.topic_in_2, self.img2_cb, 10)
            self.img1 = None
            self.img2 = None

    def image_compare(self, image1, image2):
        res = ImageCompMetrics()
        res.ssi = structural_similarity(image1, image2)
        res.psnr = peak_signal_noise_ratio(image1, image2)
        res.nmi = normalized_mutual_information(image1, image2)
        res.voi = list(variation_of_information(image1, image2))
        res.mis = mutual_info_score(image1.flatten(), image2.flatten())
        return res

    def img_cb(self, img1, img2):
        image1 = self.br.imgmsg_to_cv2(img1)
        image2 = self.br.imgmsg_to_cv2(img2)

        if image1.shape[0] != image2.shape[0]:
            image2 = image2[:image1.shape[0]]

        res = self.image_compare(image1, image2)
        res.frame1 = img1.header.frame_id
        res.frame2 = img2.header.frame_id
        res.stamp1 = img1.header.stamp
        res.stamp2 = img2.header.stamp
        res.topic1 = self.topic_in_1
        res.topic2 = self.topic_in_2

        self.pub.publish(res)

    def img1_cb(self, msg):
        self.img1 = msg
        if self.img2:
            self.img_cb(self.img1, self.img2)
            self.img1, self.img2 = None, None

    def img2_cb(self, msg):
        self.img2 = msg
        if self.img1:
            self.img_cb(self.img1, self.img2)
            self.img1, self.img2 = None, None

def main(args=None):
    rclpy.init(args=args)
    node = ImageComparer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

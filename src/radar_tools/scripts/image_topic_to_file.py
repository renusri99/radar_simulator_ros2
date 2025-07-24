import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImgSaver(Node):
    def __init__(self):
        super().__init__('image_topic_to_file')

        self.declare_parameter('topic', '/image_raw')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.br = CvBridge()
        self.image = None
        self.msg_received = False

        self.sub = self.create_subscription(Image, self.topic, self.callback, 10)
        self.get_logger().info(f'Subscribed to image topic: {self.topic}')

    def callback(self, msg):
        self.get_logger().info('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
        self.msg_received = True

    def wait_for_image(self):
        while not self.msg_received:
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_last_image(self):
        return self.image

def main(args=None):
    rclpy.init(args=args)

    node = ImgSaver()
    node.wait_for_image()
    image = node.get_last_image()

    if image is not None:
        cv2.imwrite("save.png", image)
        node.get_logger().info("Image saved as save.png")
    else:
        node.get_logger().error("No image received. Nothing saved.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os
sys.path.append(os.path.expanduser("~/Laser"))

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Create a publisher for the image topic
        self.publisher_ = self.create_publisher(Image, 'laser_image', 10)

        # Timer to call publish method at fixed intervals
        timer_period = 0.04  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # OpenCV video capture
        self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        self.bridge = CvBridge()

        self.get_logger().info('Camera publisher node has started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        frame = cv2.flip(frame, 0)

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(image_msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()

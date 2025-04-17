import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import os

sys.path.append(os.path.expanduser("~/Laser"))
from functions import laser_function as lf

class LaserTracker(Node):
    def __init__(self):
        super().__init__('laser_tracker')

        self.subscription = self.create_subscription(
            Image,
            'laser_image',
            self.listener_callback,
            10
        )

        self.publisher_ = self.create_publisher(Point, 'camera_angle', 10)
        self.bridge = CvBridge()

        # Servo angles
        self.h_angle = 96
        self.v_angle = 30
        self.kp = 0.01

        self.get_logger().info('Laser tracker node initialized.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            #self.get_logger().error(f'Could not convert image: {e}')
            return

        result = lf.laser_tracking(cv_image, self.kp, self.h_angle, self.v_angle)
        if result:
            self.h_angle, self.v_angle = result
            #self.get_logger().info(f"Laser locked: H={self.h_angle:.2f}, V={self.v_angle:.2f}")

            # Publish the result
            msg = Point()
            msg.x = float(self.h_angle)
            msg.y = float(self.v_angle)
            msg.z = 0.0  # unused
            self.publisher_.publish(msg)
        else:
            self.get_logger().info("Laser not found. No angle published.")

def main(args=None):
    rclpy.init(args=args)
    node = LaserTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

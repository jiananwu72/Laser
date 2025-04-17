import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import busio
from board import SCL, SDA
import sys
import os
sys.path.append(os.path.expanduser("~/Laser"))  # So it can use motor_function if needed

class CameraServoSubscriber(Node):
    def __init__(self):
        super().__init__('camera_servo_subscriber')

        # Setup I2C and PCA9685
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 100

        # Channel configuration (same as your laser tracker)
        self.x_channel = 0
        self.y_channel = 1
        self.horizontal_servo = servo.Servo(self.pca.channels[self.x_channel])
        self.vertical_servo = servo.Servo(self.pca.channels[self.y_channel])

        # Initialize servo positions
        self.horizontal_servo.angle = 96
        self.vertical_servo.angle = 30

        # Subscription to camera angle from laser tracker
        self.subscription = self.create_subscription(
            Point,
            'camera_angle',
            self.listener_callback,
            10
        )

        self.get_logger().info('Camera servo subscriber initialized.')

    def listener_callback(self, msg: Point):
        # Clamp angles to servo limits (0–180°)
        h_angle = max(0, min(180, msg.x))
        v_angle = max(0, min(180, msg.y))

        # Move the servos
        self.horizontal_servo.angle = h_angle
        self.vertical_servo.angle = v_angle

        self.get_logger().info(f"Moved servos to H={h_angle:.2f}, V={v_angle:.2f}")

    def destroy_node(self):
        self.pca.deinit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraServoSubscriber()
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

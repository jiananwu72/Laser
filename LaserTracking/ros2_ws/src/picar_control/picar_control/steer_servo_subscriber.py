import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo


class SteerServoSubscriber(Node):
    def __init__(self):
        super().__init__('steer_servo_subscriber')
        self.i2c_bus = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c_bus)
        self.pca.frequency = 100

        # Steering servo channel
        self.steer_channel = 14
        self.steer_servo = servo.Servo(self.pca.channels[self.steer_channel])

        # Subscription 
        self.subscription = self.create_subscription(
            Float32,
            'steer_angle',
            self.listener_callback,
            10
        )

        self.get_logger().info('Steering servo subscriber node has started.')

    def listener_callback(self, msg):
        self.steer_servo.angle = msg.data
        self.get_logger().info(f'Set steering angle: {self.steer_servo.angle:.2f} degrees')

    def destroy_node(self):
        self.steer_servo.angle = 90  
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteerServoSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()



if __name__ == '__main__':
    main()

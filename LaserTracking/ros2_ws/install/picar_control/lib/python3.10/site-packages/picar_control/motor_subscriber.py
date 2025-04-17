import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys, os
sys.path.append(os.path.expanduser('~/Laser'))
from functions import motor_function as mf

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_subscriber')
        # initialize ESC
        self.pca = mf.servo_motor_initialization()
        # mf.motor_start(self.pca)
        self.subscription = self.create_subscription(
            Float32, 'motor_speed', self.listener_callback, 10)
        self.get_logger().info('Motor subscriber ready.')

    def listener_callback(self, msg):
        speed = max(-1.0, min(1.0, msg.data))
        mf.motor_speed(self.pca, speed)
        self.get_logger().info(f'Motor speed set to {speed:.2f}')

    def destroy_node(self):
        mf.motor_speed(self.pca, 0.0)
        self.pca.deinit()
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorSubscriber()
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

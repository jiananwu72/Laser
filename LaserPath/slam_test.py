import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration

class PosePrinter(Node):
    def __init__(self):
        super().__init__('pose_printer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                "base_footprint",
                now,
                timeout=Duration(seconds=1.0)
            )
            t = transform.transform.translation
            self.get_logger().info(f"Pose: x={t.x:.2f}, y={t.y:.2f}")
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PosePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PosePrinter")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

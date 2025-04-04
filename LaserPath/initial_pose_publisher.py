#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Give some time for publishers/subscribers to connect
        time.sleep(1)
        self.publish_initial_pose()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"  # change if your fixed frame is different
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set position and orientation
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # Optionally set the covariance
        msg.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                               0, 0.25, 0, 0, 0, 0,
                               0, 0, 0.25, 0, 0, 0,
                               0, 0, 0, 0.25, 0, 0,
                               0, 0, 0, 0, 0.25, 0,
                               0, 0, 0, 0, 0, 0.25]

        self.publisher_.publish(msg)
        self.get_logger().info("Initial pose published!")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import sys
import os
import math

sys.path.append(os.path.expanduser("~/Laser"))
from functions import lidar_function as ldf


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.lidar_sub = self.create_subscription(
            Float32MultiArray,
            'scan',
            self.lidar_callback,
            10)

        self.laser_sub = self.create_subscription(
            Point,
            'camera_angle',
            self.laser_callback,
            10)

        self.steer_pub = self.create_publisher(Float32, 'steer_angle', 10)
        self.motor_pub = self.create_publisher(Float32, 'motor_speed', 10)

        self.laser_angle = None
        self.last_scan_data = []
        self.region_min = 120
        self.region_max = 240
        self.detection_threshold = 1000  # mm
        self.safe_width = 420            # mm
        self.steer_angle = 90.0  # only set to 90 once at start


        self.last_laser_status = True  # for optional logging debounce
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Obstacle avoidance node started.")

    def laser_callback(self, msg):
        self.laser_angle = msg.x
        
    def lidar_callback(self, msg):
        flat = msg.data
        scan = [(flat[i], flat[i+1]) for i in range(0, len(flat), 2)]
        scan.sort(key=lambda x: x[0])
        self.last_scan_data = scan

    def control_loop(self):
        if not self.last_scan_data:
            return

        # 1) Detect obstacles
        obstacle_angles = [a for a, d in self.last_scan_data
                           if d < self.detection_threshold]
        angle_cmd = None
        speed = 0.0

        if obstacle_angles:
            self.get_logger().info("Obstacle detected.")
            # find all gaps
            gaps = ldf.split_gaps(self.last_scan_data, self.detection_threshold)
            valid = ldf.filter_valid_spaces(gaps, self.safe_width)
            if valid:
               # build (mid_angle, servo_angle) list
                options = []
                for gap in valid:
                    mid = (gap[0][0] + gap[-1][0]) / 2
                    servo_cmd = ldf.lidar_to_steer(mid)
                    options.append((mid, servo_cmd))
                # select target: camera or center
                if self.laser_angle is not None:
                    target = 180 - self.laser_angle
                else:
                    target = 90.0
                # choose gap with servo nearest target
                best_mid, best_servo = min(options, key=lambda x: abs(x[1] - target))
                angle_cmd = best_servo
                speed=0.133
                self.get_logger().info(
                    f"Chosen gap@{best_mid:.1f}°, servo@{best_servo:.1f}° "
                    f"closest to target@{target:.1f}°"
                )

            else:
                self.get_logger().info("No valid gap—holding position.")
                speed = 0.0
        else:
            # 2) No obstacle → follow laser if available
            if self.laser_angle is not None:
                angle_cmd = 180 - self.laser_angle
                speed = 0.133
                self.get_logger().info(f"Following laser → steer {angle_cmd:.1f}")
            else:
                self.get_logger().info("No laser—holding last steer.")
                speed=0.0

        # 3) Deadband + publish
        if angle_cmd is not None and abs(angle_cmd - self.steer_angle) > 2.0:
            self.steer_angle = max(0.0, min(180.0,float(angle_cmd)))
        self.steer_pub .publish(Float32(data=self.steer_angle))
        self.motor_pub .publish(Float32(data=speed))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

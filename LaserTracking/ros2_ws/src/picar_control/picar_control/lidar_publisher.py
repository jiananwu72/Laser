import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from adafruit_rplidar import RPLidar
import threading
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')

        self.PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT_NAME, timeout=3)

        self.angle_limit = (120, 240)
        self.data_array = []
        self.lock = threading.Lock()
        self.new_data_available = False

        self.publisher_ = self.create_publisher(Float32MultiArray, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.lidar_thread = threading.Thread(target=self.process_scan)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()

    def process_scan(self):
        time.sleep(2)  # Allow time for initialization

        try:
            self.lidar.connect()
            self.lidar.start_motor()
            time.sleep(1)

            for (_, quality, angle, distance) in self.lidar.iter_measurements():
                if distance == 0:
                    continue

                if self.angle_limit[0] <= angle <= self.angle_limit[1]:
                    angle = int(angle)
                    with self.lock:
                        self.data_array.append(float(angle))
                        self.data_array.append(float(distance))

                    # Keep only the latest 60 angle-distance pairs (120 values)
                        if len(self.data_array) > 120:
                            self.data_array = self.data_array[-120:]

                        self.new_data_available = True  # Allow publish_scan to run

        except Exception as e:
            self.get_logger().error(f'LiDAR error: {e}')
        finally:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception as cleanup_error:
                self.get_logger().warn(f'Cleanup failed: {cleanup_error}')


    def publish_scan(self):
        with self.lock:
            if not self.new_data_available or not self.data_array:
                return
            data = self.data_array.copy()
            self.new_data_available = False  # Optional reset

        msg = Float32MultiArray()
        msg.data = data
        self.publisher_.publish(msg)

        #if len(data) >= 10:
            #last_10 = data[-10:]
            #last_5_points = [(last_10[i], last_10[i+1]) for i in range(0, 10, 2)]
            #self.get_logger().info(f'Published {len(data)//2} points | Last 5 angle-distance pairs: {last_5_points}')

def main(args=None):
    import contextlib  # add this import at the top
    rclpy.init(args=args)
    node = LidarPublisher()
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

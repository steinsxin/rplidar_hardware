
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from build.RPLidar import RPLidar

print("RPLidar module loaded successfully!")
M_PI= 3.141593

class RPLiadrPublisher(Node):
    def __init__(self):
        super().__init__('rplidar_node')
        self.lidar = RPLidar("/dev/ttyUSB0")
        self.publisher = self.create_publisher(LaserScan, 'scan_data', 10)
        timer_period = 0.01  # seconds | lidar: 10HZ
        self.timer = self.create_timer(timer_period, self.rpliadr_callback)
        self.count = 0

    def rpliadr_callback(self):
        range_data, intensity_data = self.lidar.get_data()
        scan_time = self.lidar.get_scan_duration()
        angle_max, angle_min = self.lidar.get_angle()
        max_distance = self.lidar.get_max_distance()
        node_count = self.lidar.get_nodes_count()
        start_scan_time = self.get_clock().now()

        # print("Range Data (first 5):", range_data[:5])
        # print("Intensity Data (first 5):", intensity_data[:5])

        scan_msg = LaserScan()

        scan_msg.header.stamp = start_scan_time.to_msg()  # 使用 ROS 2 当前时间戳
        scan_msg.header.frame_id = "laser_frame"  

        reversed = (angle_max > angle_min)
        if angle_max > angle_min :
            scan_msg.angle_min =  M_PI - angle_max
            scan_msg.angle_max =  M_PI - angle_min
        else:
            scan_msg.angle_min =  M_PI - angle_min
            scan_msg.angle_max =  M_PI - angle_max
        
        scan_msg.angle_increment = (angle_max - angle_min) / float(node_count-1)
        scan_msg.scan_time = scan_time
        scan_msg.time_increment = scan_time / float(node_count - 1)
        scan_msg.range_min = 0.15
        scan_msg.range_max = max_distance

        scan_msg.ranges = range_data
        scan_msg.intensities = intensity_data

        # scan_msg
        self.publisher.publish(scan_msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    rpliadr_publisher = RPLiadrPublisher()
    rclpy.spin(rpliadr_publisher)
    rpliadr_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
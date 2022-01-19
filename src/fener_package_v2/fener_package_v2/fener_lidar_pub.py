from rplidar import RPLidar
import numpy as np
import time

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

from os import system

NAMESPACE = "rplidar"
NODENAME = "lidar_pub"
PUBLISHERNAME = "lidar_raw"

class lidar_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.lidar = RPLidar('/dev/ttyUSB0')
        self.lidar_publisher = self.create_publisher(LaserScan, PUBLISHERNAME, 2)

        self.declare_parameter("range_min", 0.40)
        self.declare_parameter("range_max", 4.0)
        
        self.laser_msg = LaserScan()
        self.laser_msg.header.frame_id = "RP_Lidar"
        self.laser_msg.angle_min = 0.0 #TODO Bu sanırım yanlış, rplidar 358 derecede mi ne başlıyor, ama bu lidarın araç üstündeki konumuna göre şekillenecek
        self.laser_msg.angle_max = 2 * np.pi
        self.laser_msg.range_min = self.get_parameter("range_min").value
        self.laser_msg.range_max = self.get_parameter("range_max").value

        self.get_logger().info("Lidar ready and node is alive")

        self.create_timer(1, self.timer_callback)

        health = self.lidar.get_health()
        self.get_logger().info(f"Lidars health {health}") 



    def timer_callback(self):
        prev_measurement = time.time()
        for i, scan in enumerate(self.lidar.iter_scans()):
            scan = np.array(scan)[:, 1:] # 272 scan per turn
            self.laser_msg.ranges = (scan[:, 1] / 1000).tolist()
            self.laser_msg.angle_increment = 2 * np.pi / scan.shape[0]
            self.laser_msg.header.stamp = self.get_clock().now().to_msg()

            last_measurement = time.time()
            self.laser_msg.scan_time = last_measurement - prev_measurement
            prev_measurement = last_measurement

            self.lidar_publisher.publish(self.laser_msg)




def main(args=None):
    system("sudo chmod 666 /dev/ttyUSB0")
    rclpy.init(args=args)
    node = lidar_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

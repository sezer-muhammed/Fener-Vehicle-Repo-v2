#!/usr/bin/env python3
from rplidar import RPLidar
import rplidar
import numpy as np
import time

import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node

from os import system
import pickle

NAMESPACE = "rplidar"
NODENAME = "lidar_pub"
PUBLISHERNAME = "lidar_raw"



class lidar_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.history = []
        self.lidar_publisher = self.subscription = self.create_subscription(LaserScan, '/rplidar/lidar_raw', self.lidar_callback, 10)
    
    def lidar_callback(self, msg):
        max_angle = msg.angle_max
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment
        scan_time = msg.scan_time
        range_min = msg.range_min
        range_max = msg.range_max

        ranges = msg.ranges

        ThisTurn = {"max_angle":max_angle, "min_angle":min_angle, "angle_increment":angle_increment, "scan_time":scan_time, "range_min":range_min, "range_max":range_max, "ranges":ranges}
        self.history.append(ThisTurn)
        
        print(len(self.history))

        if len(self.history) == 360:
            with open("laser_data.pickle", "wb") as fp:
                pickle.dump(self.history, fp)
            exit()


def main(args=None):
    system("sudo chmod 666 /dev/ttyUSB0")
    rclpy.init(args=args)
    node = lidar_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

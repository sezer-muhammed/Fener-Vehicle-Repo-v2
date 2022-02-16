#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


NODENAME = "baselink_to_odom"
PUBLISHERNAME = "tf" #define edilmesi istenen değerler


class odom_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME)
        self.br = TransformBroadcaster(self)

        self.create_subscription(Imu, "/bno055/bno_data", self.bno055_callback, 1)
        self.create_timer(1/50, self.calculate_transform)

    def calculate_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        self.counter = self.counter + self.counter_sign / 100

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
  
        t.transform.rotation.x = self.orientation.x
        t.transform.rotation.y = self.orientation.y
        t.transform.rotation.z = self.orientation.z
        t.transform.rotation.w = self.orientation.w

        self.br.sendTransform(t)

    def bno055_callback(self, msg):
        self.orientation = msg.orientation

def main(args=None):
    rclpy.init(args=args)
    node = odom_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()




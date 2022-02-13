#!/usr/bin/env python3
import rclpy #import ROS2
from rclpy.node import Node #import ROS2's Node
from std_msgs.msg import Int64MultiArray #will be deleted
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import numpy as np
import time
import serial
from os import system
import math


NAMESPACE = "Arduino"
NODENAME = "Communication"
PUBLISHERNAME = "encoder_data"

KEY_ANGLE_MULTIPLIER = 40
KEY_ANGLE_BNO_MULTIPLIER = 25
KEY_SPEED_COEFF = 200

class MyNode(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)
        self.get_logger().info("The Node has been started.")

        self.ard = serial.Serial(port = "/dev/ttyACM0", baudrate = 230400, timeout = 1)
        time.sleep(2)

        self.keyboard_packet = [0, 0, 0, 1]
        self.keyboard_commands = [0, 1, 2, 3]

        self.old_encoder = np.array([0, 0, 0, 0])
        self.encoder = np.array([0, 0, 0, 0])
        self.total_encoder = np.array([0, 0, 0, 0], np.int64)
        self.total_diff = np.array([0, 0, 0, 0])

        self.bno055_heading = 0
 
        self.encoder_msg = Int64MultiArray()

        self.key_speed = 0
        self.key_angle = 1500
        self.aim_angle = 0
        self.bno055_heading = 0

        self.create_subscription(Imu, "/bno055/bno_data", self.bno055_callback, 1) #TODO Change it according to IMU PUB
        self.create_subscription(Twist, "/cmd_vel", self.keyboard_callback, 1) 

        self.encoder_publisher = self.create_publisher(Int64MultiArray, PUBLISHERNAME, 1)

        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        
        self.keyboard_packet[0:3] = [self.key_speed, min(3000, max(0, self.key_angle + (self.aim_angle - self.bno055_heading))), self.bno055_heading%360]
        print(self.keyboard_packet[:2])
        self.old_encoder = self.encoder
        self.encoder = self.send_to_arduino(self.keyboard_commands, self.keyboard_packet)

        diff = self.encoder - self.old_encoder
        diff[diff > 0] = 0
        diff[diff < 0] = 254

        self.total_diff = self.total_diff + diff
        self.total_encoder = self.encoder + self.total_diff
        self.encoder_msg.data = self.total_encoder.tolist()

        self.encoder_publisher.publish(self.encoder_msg)

        debug_data = self.ard.readline().decode()


    def send_to_arduino(self, commands, values):

            msg = bytearray()
            for command, value in zip(commands, values):
                    first_byte = value % 256
                    second_byte = value // 256
                    command_byte = command * 32
                    msg.append(first_byte)
                    msg.append(second_byte + command_byte)

            self.ard.write(msg)

            output = np.array(list(self.ard.read(size = 4))) - 1

            return output

    def keyboard_callback(self, msg):
        self.aim_angle = self.bno055_heading
        self.key_speed = int(msg.linear.x * KEY_SPEED_COEFF + 2048)
        self.key_angle = int(1500 - msg.angular.z * KEY_ANGLE_MULTIPLIER) # mid is 1500
        

    def bno055_callback(self, msg):
        equation_1 = 2 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
        equation_2 = 1 - 2 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        self.bno055_heading = math.atan2(equation_1, equation_2) #TODO TEST İF İTS CORRECT


def main(args=None):
    system("sudo chmod 666 /dev/ttyACM0")
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

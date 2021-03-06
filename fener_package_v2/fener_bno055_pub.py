#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
import adafruit_bno055
from sensor_msgs.msg import Imu #gerekli kütüphaneleri yükle

NAMESPACE = "bno055"
NODENAME = "BNO055_sensor"
PUBLISHERNAME = "bno_data" #define edilmesi istenen değerler


class BNO055_sensor(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.i2c = board.I2C() #(1)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c) #(1)
         
        self.imu_res = Imu() #(2)
        self.imu_res.header.frame_id = "BNO055" #(3)

        self.bno_data_publisher = self.create_publisher(Imu, PUBLISHERNAME, 2) #(4)
    
        self.create_timer(1/20, self.timer_callback) #(5)

        self.get_logger().info("BNO055 Started") #(6)
    
        
    def quaternion_value(self):

        self.imu_res.orientation.w = self.sensor.quaternion[0]
        self.imu_res.orientation.x = self.sensor.quaternion[1]
        self.imu_res.orientation.y = self.sensor.quaternion[2]
        self.imu_res.orientation.z = self.sensor.quaternion[3]
    
    def linear_acceleration_value(self):

        self.imu_res.linear_acceleration.x = self.sensor.linear_acceleration[0]
        self.imu_res.linear_acceleration.y = self.sensor.linear_acceleration[1]
        self.imu_res.linear_acceleration.z = self.sensor.linear_acceleration[2]
    
    def angular_velocity_value(self):
    
        self.imu_res.angular_velocity.x = self.sensor.gyro[0]
        self.imu_res.angular_velocity.y = self.sensor.gyro[1]
        self.imu_res.angular_velocity.z = self.sensor.gyro[2]
    
    def timer_callback(self):
        
        self.quaternion_value() #(1)
        self.linear_acceleration_value() #(2)
        self.angular_velocity_value() #(3)

        self.imu_res.header.stamp = self.get_clock().now().to_msg() #(4)
        
        self.bno_data_publisher.publish(self.imu_res) #(5)


  

def main(args=None):
    rclpy.init(args=args)
    node = BNO055_sensor()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()




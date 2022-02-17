#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from numpy import cov
from sensor_msgs.msg import Imu

NAMESPACE = "calibrator"
NODENAME = "kalibrator"
PUBLISHERNAME = "calibrator_data"

class kalibrator(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace=NAMESPACE)

        self.i2c = board.I2C()  # (1)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        self.imu_tmp = Imu()

        self.bno_data_publisher = self.create_publisher(Imu, "/bno055/calibrator_data", 2)  # (4)

        self.create_timer(1 / 20, self.timer_callback)  # (5)

        self.get_logger().info("Calibrator Started")

        self.count = 1
        self.imu_or_x = []
        self.imu_or_y = []
        self.imu_or_z = []
        self.imu_la_x = []
        self.imu_la_y = []
        self.imu_la_z = []
        self.imu_av_x = []
        self.imu_av_y = []
        self.imu_av_z = []

    def covariance(self):
        self.imu_tmp.orientation_covariance = [cov(self.imu_or_x, self.imu_or_x), cov(self.imu_or_x, self.imu_or_y), cov(self.imu_or_x, self.imu_or_z),
                                               cov(self.imu_or_y, self.imu_or_x), cov(self.imu_or_y, self.imu_or_y), cov(self.imu_or_y, self.imu_or_z),
                                               cov(self.imu_or_z, self.imu_or_x), cov(self.imu_or_z, self.imu_or_y), cov(self.imu_or_z, self.imu_or_z)]

        self.imu_tmp.linear_acceleration_covariance = [cov(self.imu_la_x, self.imu_la_x), cov(self.imu_la_x, self.imu_la_y), cov(self.imu_la_x, self.imu_la_z),
                                                       cov(self.imu_la_y, self.imu_la_x), cov(self.imu_la_y, self.imu_la_y), cov(self.imu_la_y, self.imu_la_z),
                                                       cov(self.imu_la_z, self.imu_la_x), cov(self.imu_la_z, self.imu_la_y), cov(self.imu_la_z, self.imu_la_z)]

        self.imu_tmp.angular_velocity_covariance = [cov(self.imu_av_x, self.imu_av_x), cov(self.imu_av_x, self.imu_av_y), cov(self.imu_av_x, self.imu_av_z),
                                                    cov(self.imu_av_y, self.imu_av_x), cov(self.imu_av_y, self.imu_av_y), cov(self.imu_av_y, self.imu_av_z),
                                                    cov(self.imu_av_z, self.imu_av_x), cov(self.imu_av_z, self.imu_av_y), cov(self.imu_av_z, self.imu_av_z)]

    def add_covariance(self):
        self.imu_or_x.append(self.sensor.quaternion.x)
        self.imu_or_y.append(self.sensor.quaternion.y)
        self.imu_or_z.append(self.sensor.quaternion.z)
        self.imu_la_x.append(self.sensor.linear_acceleration.x)
        self.imu_la_y.append(self.sensor.linear_acceleration.y)
        self.imu_la_z.append(self.sensor.linear_acceleration.z)
        self.imu_av_x.append(self.sensor.gyro.x)
        self.imu_av_y.append(self.sensor.gyro.y)
        self.imu_av_z.append(self.sensor.gyro.z)

    
    def timer_callback(self):

        if self.count == 0:
            self.covariance()
            self.count = -1

        elif self.count > 12000:
            self.count = 0

        elif self.count > 0 and self.count <= 12000:
            self.add_covariance()
            self.count += 1
        
        if self.count == -1:
            self.bno_data_publisher.publish(self.imu_tmp)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = kalibrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy #ROS2 Python kütüphanesi
from rclpy.node import Node #ROS2'nin Node objesi
from sensor_msgs.msg import Imu #Geometri mesajlarından istenen obje veya objeler

import numpy as np #NumPY kütüphanesi
import csv

NODENAME = "BNO055_covariance_calculator"
NAMESPACE = "bno055"

class data_saver(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.limit = 12000
        self.data_history_orientation = np.zeros((3, self.limit))
        self.data_history_angular = np.zeros((3, self.limit))
        self.data_history_linear = np.zeros((3, self.limit))

        self.counter = 0

        self.detection_Left = self.create_subscription(Imu, "bno_data", self.Callback, 2)

    def Callback(self, msg):
        
        if self.counter == self.limit:
            self.save_covariance()
            exit()

        self.data_history_orientation[0][self.counter] = msg.orientation.x
        self.data_history_orientation[1][self.counter] = msg.orientation.y
        self.data_history_orientation[2][self.counter] = msg.orientation.z

        self.data_history_angular[0][self.counter] = msg.angular_velocity.x
        self.data_history_angular[1][self.counter] = msg.angular_velocity.y
        self.data_history_angular[2][self.counter] = msg.angular_velocity.z

        self.data_history_linear[0][self.counter] = msg.linear_acceleration.x
        self.data_history_linear[1][self.counter] = msg.linear_acceleration.y
        self.data_history_linear[2][self.counter] = msg.linear_acceleration.z

        self.counter += 1

        print(f"Total IMU Data Arrived: {self.counter}".ljust(60), end = "\r")

    def save_covariance(self):
        print("")
        print("Orientation:")
        print(np.cov(self.data_history_orientation))
        print("")
        print("Angular")
        print(np.cov(self.data_history_angular))
        print("")
        print("Linear")
        print(np.cov(self.data_history_linear))

        header = ['Orientation_xyz_3x3', 'Angular_xyz_3x3', 'Linear_xyz_3x3']

        with open('sensor_results/IMU_covariance.csv', 'w', encoding='UTF8') as f:
            writer = csv.writer(f)

            writer.writerow(header)


            for orientation, angular, velocity in zip(np.cov(self.data_history_orientation).flatten().tolist(), np.cov(self.data_history_angular).flatten().tolist(), np.cov(self.data_history_linear).flatten().tolist()):
                data = [orientation, angular, velocity]
                writer.writerow(data)


def main(args=None):
    rclpy.init(args=args)
    node = data_saver()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

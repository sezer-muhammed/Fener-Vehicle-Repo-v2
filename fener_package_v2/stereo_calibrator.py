import rclpy #ROS2 Python kütüphanesi
from rclpy.node import Node #ROS2'nin Node objesi
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose #Görüntü mesajlarından istenen obje veya objeler
from geometry_msgs.msg import Pose2D #Geometri mesajlarından istenen obje veya objeler

import cv2 #CPU üzerinde daha komplex görüntü işleme işlemleri OpenCV kütüphanesi
import numpy as np #NumPY kütüphanesi
import message_filters as mf
import csv

SHAPE = [1640, 922] #Paylaşılacak görüntünün çözünürlüğü
NAMESPACE = "csi_cam" #Bu Nod için kullanılacak isim uzayı
NODENAME = "detetion_saver" #Bu Nod için kullanılacak isim
DETECTION_PUBLISHERNAME = "detections" #Model tespitlerinin paylaşılacağı başlığın ismi
DOMINANT_CAMERA = "Right" #Görüntü paylaşırken ve konumu bulunurken referans alınacak kamera

class data_saver(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.left_x = 0
        self.right_x = 0

        self.header = header = ["Left x", "Right x", "Pixel diff", "output"]
        self.data = []

        self.detection_Left = self.create_subscription(Detection2DArray, "left_camera/" + DETECTION_PUBLISHERNAME, self.Callback_left, 2)
        self.detection_Right = self.create_subscription(Detection2DArray, "right_camera/" + DETECTION_PUBLISHERNAME, self.Callback_right, 2)

        self.timer = self.create_timer(1/10, self.saver)



    def saver(self):
        command = input()
        try:
            distance = float(command)
            sample = [self.left_x, self.right_x, 1 / (self.right_x - self.left_x), distance]
            self.data.append(sample)
            print(f"{self.left_x}".ljust(20) + f"{self.right_x}".ljust(20), f"{1 / (self.right_x - self.left_x)}".ljust(20) + f"  || {distance}")
        except:
            print(f"{self.left_x}".ljust(20) + f"{self.right_x}".ljust(20), f"{self.right_x - self.left_x}".ljust(20))

        if command == "finish":
            with open('distance.csv', 'w', encoding='UTF8', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.header)
                writer.writerows(self.data)
            exit()

    def Callback_left(self, msg):
        for detection in msg.detections:
            self.left_x = detection.bbox.center.x - (SHAPE[0]//2)
            ID = detection.results[0].id
             

    def Callback_right(self, msg):
        for detection in msg.detections:
            self.right_x = detection.bbox.center.x - (SHAPE[0]//2)

def main(args=None):
    rclpy.init(args=args)
    node = data_saver()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import jetson.inference
import jetson.utils

import time
import cv2
import numpy as np
from cv_bridge import CvBridge

TARGET_SHAPE = [480, 360]
NAMESPACE = "csi_cam"
NODENAME = "solocam_publisher"
PUBLISHERNAME = "solocam_image_raw"
DOMINANT_CAMERA = "right"

class solo_cam_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)
        
        self.image_publisher = self.create_publisher(Image, PUBLISHERNAME, 2)

        self.declare_parameter("publish_image", False)
        self.declare_parameter("detect_objects", False)
        self.declare_parameter("estimate_object_distance", False)
        self.declare_parameter("estimate_object_id", -1)
        self.declare_parameter("image_width", 1640)
        self.declare_parameter("image_height", 922)


        self.img_shape = [self.get_parameter("image_width").value, self.get_parameter("image_height").value]

        self.camera_right = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://1")
        self.camera_right.Open()

        self.camera_left = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://0")
        self.camera_left.Open()

        self.bridge = CvBridge()
        self.get_logger().info("Camera ready and node is alive with two cameras")

        self.create_timer(1/60, self.capture_images)

        if (self.get_parameter("publish_image").value):
            self.create_timer(1/20, self.publish_image)

        if (self.get_parameter("detect_objects").value):
            self.create_timer(1/20, self.detect_objects)

        if (self.get_parameter("estimate_object_distance").value and self.get_parameter("detect_objects").value):
            self.create_timer(1/20, self.estimate_object_distance)
            self.target_object = self.get_parameter("estimate_object_id").value

    def detect_objects(self):
        pass

    def estimate_object_distance(self):
        pass

    def capture_images(self):
        self.img_right, w,  h = self.camera_right.CaptureRGBA()
        self.img_left, w,  h = self.camera_left.CaptureRGBA()

    def publish_image(self):

        if DOMINANT_CAMERA == "right":
            self.small_img = self.resize(self.img_right, TARGET_SHAPE)
            self.small_opencv_img = np.array(jetson.utils.cudaToNumpy(self.small_img)[:,:,:3], np.uint8)

        elif DOMINANT_CAMERA == "left":
            self.small_img = self.resize(self.img_left, TARGET_SHAPE)
            self.small_opencv_img = np.array(jetson.utils.cudaToNumpy(self.small_img)[:,:,:3], np.uint8)

        jetson.utils.cudaDeviceSynchronize()

        msg = self.bridge.cv2_to_imgmsg(self.small_opencv_img, encoding="rgb8")
        msg.header.frame_id = "camera" #TODO bu böyle kalmamalı
        self.image_publisher.publish(msg)


    def resize(self, img, new_shape):
        resized_img = jetson.utils.cudaAllocMapped(width=new_shape[0], height=new_shape[1], format=img.format)
        jetson.utils.cudaResize(img, resized_img)
        return resized_img

  

def main(args=None):
    rclpy.init(args=args)
    node = solo_cam_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()
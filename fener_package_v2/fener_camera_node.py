import cv_bridge
import rclpy
from rclpy.node import Node
import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from vision.msgs
from cv_bridge import CvBridge

TARGET_SHAPE = [480, 360]
NAMESPACE = "csi_cam"
NODENAME = "solocam_publisher"
PUBLISHERNAME = "solocam_image_raw"

class solo_cam_publisher(Node):
  def __init__(self):
    super().__init__(NODENAME, namespace = NAMESPACE)
    self.image_publisher = self.create_publisher(CompressedImage, PUBLISHERNAME, 2)

    self.declare_parameter("publish_image", False)
    self.declare_parameter("detect_objects", False)
    self.declare_parameter("estimate_object_distance", False)
    self.declare_parameter("estimate_object_id", -1)
    self.declare_parameter("image_width", 960)
    self.declare_parameter("image_height", 720)

    self.target_shape = [self.get_parameter("image_width").value, self.get_parameter("image_height").value]


    self.camera = jetson.utils.gstCamera(1640, 1232, "csi://0")
    self.camera.Open()

    self.bridge = cv_bridge.CvBridge()
    self.get_logger().info("Camera ready and node is alive")

    if (self.get_parameter("publish_image").value):
        self.create_timer(1/20, self.publish_image)

    if (self.get_parameter("detect_objects").value):
        self.create_timer(1/20, self.detect_objects)

    if (self.get_parameter("estimate_object_distance").value):
        self.create_timer(1/20, self.estimate_object_distance)
        self.target_object = self.get_parameter("estimate_object_id").value

    def detect_objects(self):
        pass

    def estimate_object_distance(self):
        pass

    def publish_image(self):
        self.img, w,  h = self.camera.CaptureRGBA()
        self.small_img = self.resize(self.img, TARGET_SHAPE)
        self.small_opencv_img = jetson.utils.cudaToNumpy(self.small_img)
        jetson.utils.cudaDeviceSynchronize()
        msg = self.bridge.cv2_to_compressed_imgmsg(self.small_opencv_img)
        msg.header.frame = "camera"
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
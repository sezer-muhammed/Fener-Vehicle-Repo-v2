import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

import jetson.inference
import jetson.utils

import time
import cv2
import numpy as np
from cv_bridge import CvBridge

"""
Image publish, detection ve tracking kapalı iken de çalışabilir hale getirilmeli
- imagepublish açık olan tek parametre mi diye kontol edilip ardından tek ise publisher fonksiyonu çağırılabilir, hem bu durumda çözünürlüğü de arttırılmış olarak paylaşım yapılır

Tracking callbak'i yazılmalı
"""


TARGET_SHAPE = [480, 360]
NAMESPACE = "csi_cam"
NODENAME = "image_node"
IMAGE_PUBLISHERNAME = "dominant_image_raw"
DETECTION_PUBLISHERNAME = "detections"
TRACKING_PUBLISHERNAME = "tracked_detection"
DOMINANT_CAMERA = "Right"

class solo_cam_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE)

        self.declare_parameter("publish_image", False)
        self.declare_parameter("image_overlay", True)
        self.declare_parameter("detect_objects", False)
        self.declare_parameter("track_object", False)
        self.declare_parameter("estimate_object_id", -1)
        self.declare_parameter("image_width", 1640)
        self.declare_parameter("image_height", 922)


        if self.get_parameter("detect_objects").value + self.get_parameter("track_object").value == 2:
            self.get_logger().info(f"detect_objects and track_object cannot be online at the same time.")
            exit()

        self.get_logger().info(f"Image Publisher: {1} | Detection: {1} | Tracking: {1}")

        self.img_shape = [self.get_parameter("image_width").value, self.get_parameter("image_height").value]

        self.camera_right = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://1")
        self.camera_right.Open()

        self.camera_left = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://0")
        self.camera_left.Open()

        self.bridge = CvBridge()
        self.get_logger().info("Camera ready and node is alive with two cameras")

        if (self.get_parameter("publish_image").value):
            self.image_publisher = self.create_publisher(Image, IMAGE_PUBLISHERNAME, 2)

        if (self.get_parameter("detect_objects").value):

            self.left_detections_publisher = self.create_publisher(Detection2DArray, "left_camera/" + DETECTION_PUBLISHERNAME, 2)
            self.right_detections_publisher = self.create_publisher(Detection2DArray, "right_camera/" + DETECTION_PUBLISHERNAME, 2)

            self.network = jetson.inference.detectNet(argv=[
            '--model=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/ssd-mobilenet.onnx', 
            '--labels=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/labels.txt', 
            '--input-blob=input_0', 
            '--output-cvg=scores', 
            '--output-bbox=boxes'])

            self.create_timer(1/30, self.detect_objects)

        if (self.get_parameter("track_object").value):

            self.detections_publisher = self.create_publisher(Detection2D, "left_camera/" + TRACKING_PUBLISHERNAME, 2)
            self.detections_publisher = self.create_publisher(Detection2D, "right_camera/" + TRACKING_PUBLISHERNAME, 2)
            self.target_object = self.get_parameter("estimate_object_id").value

            self.create_timer(1/30, self.track_object)

    def detect_objects(self):
        self.capture_images()

        detections_left_msg = Detection2DArray()
        detections_left_msg.header.frame_id = "Camera Left"
        detections_left_msg.header.stamp = self.get_clock().now().to_msg()

        detections_right_msg = Detection2DArray()
        detections_right_msg.header.frame_id = "Camera Right"
        detections_right_msg.header.stamp = self.get_clock().now().to_msg()


        if self.get_parameter("publish_image").value and not self.get_parameter("image_overlay").value:
            if DOMINANT_CAMERA == "Left":
                self.publish_image(self.img_left)
            elif DOMINANT_CAMERA == "Right":
                self.publish_image(self.img_right)

        for image, side in [(self.img_left, "Left"), (self.img_right, "Right")]:
            detections = self.network.Detect(image)
            
            if self.get_parameter("publish_image").value and side == DOMINANT_CAMERA and self.get_parameter("image_overlay").value:
                self.publish_image(image)

            for single_detection in detections:
                detection2D_msg = Detection2D()
                detection2D_msg.header.frame_id = "Camera " + side
                detection2D_msg.header.stamp = self.get_clock().now().to_msg()

                results = ObjectHypothesisWithPose()
                results.id = str(single_detection.ClassID)
                results.score = single_detection.Confidence

                center = Pose2D()
                center.x = single_detection.Center[0]
                center.y = single_detection.Center[1]
                center.theta = 0.0

                bbox = BoundingBox2D()
                bbox.center = center
                bbox.size_x = single_detection.Width
                bbox.size_y = single_detection.Height
                

                detection2D_msg.results.append(results)
                detection2D_msg.bbox = bbox

                if side == "Right":
                    detections_right_msg.detections.append(detection2D_msg)
                elif side == "Left":
                    detections_left_msg.detections.append(detection2D_msg)

        self.right_detections_publisher.publish(detections_right_msg)
        self.left_detections_publisher.publish(detections_left_msg)


    def track_object(self):
        pass

    def capture_images(self):
        self.img_right, w,  h = self.camera_right.CaptureRGBA()
        self.img_left, w,  h = self.camera_left.CaptureRGBA()

    def publish_image(self, capsulated_img):

        small_img = self.resize(capsulated_img, TARGET_SHAPE)
        small_opencv_img = np.array(jetson.utils.cudaToNumpy(small_img)[:,:,:3], np.uint8)

        jetson.utils.cudaDeviceSynchronize()

        msg = self.bridge.cv2_to_imgmsg(small_opencv_img, encoding="rgb8")
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
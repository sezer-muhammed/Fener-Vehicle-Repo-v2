import rclpy #ROS2 Python kütüphanesi
from rclpy.node import Node #ROS2'nin Node objesi
from sensor_msgs.msg import Image #Sensör mesajlarından istenen obje veya objeler
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose #Görüntü mesajlarından istenen obje veya objeler
from geometry_msgs.msg import Pose2D #Geometri mesajlarından istenen obje veya objeler

import jetson.inference #Yapay Zeka işlemleri için Jetson inference kütüphanesi
import jetson.utils #GPU üzerinde görüntü ve kamera işlemleri için Jetson utils kütüphanesi

import time #Süre ölçme ve uyku işlemleri için time kütüphanesi
import cv2 #CPU üzerinde daha komplex görüntü işleme işlemleri OpenCV kütüphanesi
import numpy as np #NumPY kütüphanesi
from cv_bridge import CvBridge #OpenCV görselini ROS2 Image formatına çevirmek için CvBridge objesi

"""
TODO Pür kamera paylaşımı
"""


TARGET_SHAPE = [480 * 2, 360] #Paylaşılacak görüntünün çözünürlüğü
NAMESPACE = "csi_cam" #Bu Nod için kullanılacak isim uzayı
NODENAME = "image_node" #Bu Nod için kullanılacak isim
IMAGE_PUBLISHERNAME = "dominant_image_raw" #Görüntünün paylaşılacağı başlığın ismi
DETECTION_PUBLISHERNAME = "detections" #Model tespitlerinin paylaşılacağı başlığın ismi
TRACKING_PUBLISHERNAME = "tracked_detection" #Takip edilen objenin paylaşılacağı başlığın ismi
DOMINANT_CAMERA = "Right" #Görüntü paylaşırken ve konumu bulunurken referans alınacak kamera

class solo_cam_publisher(Node):
    def __init__(self):
        super().__init__(NODENAME, namespace = NAMESPACE) #Nod objesini verilen nod ismi ve isim uzayı ile başlat

        self.declare_parameter("publish_image", False) #Görüntü paylaşımını aktive eden parametre
        self.declare_parameter("image_overlay", True) #Paylaşılan görüntüde model tespitlerinin çizdirilmesini 
                                                      #aktive eden parametre
        self.declare_parameter("detect_objects", False) #Obje tespitleri paylaşma modunu aktive eden parametre
        self.declare_parameter("track_object", False) #Objeyi takip etme modunu aktive eden parametre
        self.declare_parameter("target_object_id", -1) #Takip edilmesi istenen objenin ID'si
        self.declare_parameter("image_width", 1640) #Kameradan alınacak görüntünün yatay çözünürlüğü
        self.declare_parameter("image_height", 922) #kameradan alınacak görüntünün dikey çözünürlüğü


        if self.get_parameter("detect_objects").value + self.get_parameter("track_object").value == 2: #Aynı anda obje tespiti ve takibi isteniyorsa
            self.get_logger().info(f"detect_objects and track_object cannot be online at the same time.") #Hata mesajını gir ve kodu kapat
            exit()

        self.img_shape = [self.get_parameter("image_width").value, self.get_parameter("image_height").value] #Kameradan alınacak görüntü çözünürlüğünü tutan liste

        self.camera_right = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://1") #1 numaralı CSI Kameradan verilen çözünürlükte görüntü alan kamera objesini oluştur
        self.camera_right.Open() #Kamera objesini başlat

        self.camera_left = jetson.utils.gstCamera(self.img_shape[0], self.img_shape[1], "csi://0") #0 numaralı CSI Kameradan verilen çözünürlükte görüntü alan kamera objesini oluştur
        self.camera_left.Open() #Kamera objesini başlat

        self.bridge = CvBridge() #OpenCV ve ROS2 Image mesaj tipi için dönüşüm objesini oluştur
        self.get_logger().info("Camera ready and node is alive with two cameras") #Tüm işlemler başarı ile yapıldıysa Loglara bu durumu kaydet

        if (self.get_parameter("publish_image").value): #Görüntü paylaşımı isteniyorsa...
            self.image_publisher = self.create_publisher(Image, IMAGE_PUBLISHERNAME, 2) #Image veri tipini sabitlerde tanımlanmış başlık ismi ile buffer boyutu 2 olacak şekilde paylaş

        if (self.get_parameter("detect_objects").value): #Obje tespiti isteniyorsa...
            self.left_detections_publisher = self.create_publisher(Detection2DArray, "left_camera/" + DETECTION_PUBLISHERNAME, 2) #Sol kamera için Detection2DArray veri tipini sabitlerde tanımlanmış başlık ismi ile buffer boyutu 2 olacak şekilde paylaş
            self.right_detections_publisher = self.create_publisher(Detection2DArray, "right_camera/" + DETECTION_PUBLISHERNAME, 2) #Sağ kamera için Detection2DArray veri tipini sabitlerde tanımlanmış başlık ismi ile buffer boyutu 2 olacak şekilde paylaş

            self.network = jetson.inference.detectNet(argv=[
            '--model=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/ssd-mobilenet.onnx', 
            '--labels=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/labels.txt', 
            '--input-blob=input_0', 
            '--output-cvg=scores', 
            '--output-bbox=boxes'])

            self.create_timer(1/30, self.detect_objects)

        if (self.get_parameter("track_object").value):

            self.left_tracking_publisher = self.create_publisher(Detection2D, "left_camera/" + TRACKING_PUBLISHERNAME, 2)
            self.right_tracking_publisher = self.create_publisher(Detection2D, "right_camera/" + TRACKING_PUBLISHERNAME, 2)
            self.target_object = self.get_parameter("target_object_id").value

            self.frame_data = [[0, 0] + self.img_shape + [0],
                                [0, 0] + self.img_shape + [0]] #Left ve Right kameraların frame verileri

            self.last_success_right = Detection2D()
            self.last_success_left = Detection2D()

            self.network = jetson.inference.detectNet(argv=[
            '--model=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/ssd-mobilenet.onnx', 
            '--labels=/home/fener/fener_vehicle/src/Fener-Vehicle-Repo-v2/fener_package_v2/labels.txt', 
            '--input-blob=input_0', 
            '--output-cvg=scores', 
            '--output-bbox=boxes'])

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
            self.publish_image(self.img_left, self.img_right)


        for image, side in [(self.img_left, "Left"), (self.img_right, "Right")]:
            detections = self.network.Detect(image)
            
            if self.get_parameter("publish_image").value and side == "Right" and self.get_parameter("image_overlay").value:
                self.publish_image(self.img_left, self.img_right)

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
        self.capture_images()

        tracking_left_msg = Detection2D()
        tracking_left_msg.header.frame_id = "Camera Left"
        tracking_left_msg.header.stamp = self.get_clock().now().to_msg()

        tracking_right_msg = Detection2D()
        tracking_right_msg.header.frame_id = "Camera Right"
        tracking_right_msg.header.stamp = self.get_clock().now().to_msg()

        if self.get_parameter("publish_image").value and not self.get_parameter("image_overlay").value:
            self.publish_image(self.img_left, self.img_right)


        image_cropped_left = self.crop(self.img_left, self.frame_data[0][0:4])
        image_cropped_right = self.crop(self.img_right, self.frame_data[1][0:4])

        success = [-1, -1]

        for image, side, row in [(image_cropped_left, "Left", 0), (image_cropped_right, "Right", 1)]:
            
            detections = self.network.Detect(image)

            for single_detection in detections:
                if single_detection.ClassID == self. target_object:

                    results = ObjectHypothesisWithPose()
                    results.id = str(single_detection.ClassID)
                    results.score = single_detection.Confidence

                    center = Pose2D()
                    center.x = single_detection.Center[0] + self.frame_data[row][0]
                    center.y = single_detection.Center[1] + self.frame_data[row][1]
                    center.theta = 0.0

                    bbox = BoundingBox2D()
                    bbox.center = center
                    bbox.size_x = single_detection.Width
                    bbox.size_y = single_detection.Height
                    

                    if side == "Right":
                        success[1] = 1
                        tracking_right_msg.results.append(results)
                        tracking_right_msg.bbox = bbox
                        tracking_left_msg.is_tracking = True
                        tracking_left_msg.tracking_id = f"left {single_detection.ClassID}"
                        self.last_success_right = tracking_right_msg
                    elif side == "Left":
                        success[0] = 1
                        tracking_left_msg.results.append(results)
                        tracking_left_msg.bbox = bbox
                        tracking_left_msg.is_tracking = True
                        tracking_left_msg.tracking_id = f"right {single_detection.ClassID}"
                        self.last_success_left = tracking_left_msg
                    break
                else:
                    pass    

        if self.get_parameter("publish_image").value and self.get_parameter("image_overlay").value:
            self.publish_image(image_cropped_left, image_cropped_right)

        self.calculate_new_frame_data(success, self.last_success_left, self.last_success_right)

        self.right_tracking_publisher.publish(tracking_right_msg)
        self.left_tracking_publisher.publish(tracking_left_msg)

    def calculate_new_frame_data(self, success, msg_left, msg_right):

        for i, msg in enumerate([msg_left, msg_right]):
            self.frame_data[i][4] = max(0, min(success[i] + self.frame_data[i][4], 5))
            if self.frame_data[i][4] == 0:
                gap_x = self.img_shape[0]
                gap_y = self.img_shape[1]
            else:
                gap_x = msg.bbox.size_x//2 + 480 - self.frame_data[i][4] * 70
                gap_y = msg.bbox.size_y//2 + 480 - self.frame_data[i][4] * 70

            x_min = max(msg.bbox.center.x - gap_x, 0)
            x_max = min(msg.bbox.center.x + gap_x, self.img_shape[0])
            y_min = max(msg.bbox.center.y - gap_y, 0)
            y_max = min(msg.bbox.center.y + gap_y, self.img_shape[1])
            
            self.frame_data[i][0:4] = [int(x_min), int(y_min), int(x_max), int(y_max)]

    def capture_images(self):
        self.img_right, w,  h = self.camera_right.CaptureRGBA()
        self.img_left, w,  h = self.camera_left.CaptureRGBA()

    def publish_image(self, capsulated_img_left, capsulated_img_right):

        imgOutput = jetson.utils.cudaAllocMapped(width=capsulated_img_left.width + capsulated_img_right.width, 
                                                 height=max(capsulated_img_left.height, capsulated_img_right.height), 
                                                 format=capsulated_img_left.format)

        jetson.utils.cudaOverlay(capsulated_img_left, imgOutput, 0, 0)
        jetson.utils.cudaOverlay(capsulated_img_right, imgOutput, capsulated_img_left.width, 0)

        small_img = self.resize(imgOutput, TARGET_SHAPE)
        small_opencv_img = np.array(jetson.utils.cudaToNumpy(small_img)[:,:,:3], np.uint8)

        jetson.utils.cudaDeviceSynchronize()

        msg = self.bridge.cv2_to_imgmsg(small_opencv_img, encoding="rgb8")
        msg.header.frame_id = "stereo vision" #TODO bu böyle kalmamalı
        self.image_publisher.publish(msg)

    def resize(self, img, new_shape):
        resized_img = jetson.utils.cudaAllocMapped(width=new_shape[0], height=new_shape[1], format=img.format)
        jetson.utils.cudaResize(img, resized_img)
        return resized_img

    def crop(self, img, bbox):
        imgOutput = jetson.utils.cudaAllocMapped(width= (bbox[2] - bbox[0]),
                                                height= (bbox[3] - bbox[1]),
                                                format=img.format)
        jetson.utils.cudaCrop(img, imgOutput, bbox)

        return imgOutput                       

def main(args=None):
    rclpy.init(args=args)
    node = solo_cam_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if  __name__ == "__main__":
    main()

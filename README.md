
# Fener Vehicle ROS 2 (v2)

![Fener vehicle side view](https://bmeqhxsikltbwjf8.public.blob.vercel-storage.com/20210912_184553.jpg)

![ROS 2](https://img.shields.io/badge/ROS2-Eloquent-blue)
![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Jetson-green)
![Python](https://img.shields.io/badge/Python-3.x-blueviolet)
![Hardware](https://img.shields.io/badge/Hardware-BNO055%20%7C%20RP--LiDAR%20%7C%20CSI%20Stereo-black)
![Status](https://img.shields.io/badge/Status-Experimental-orange)

ROS 2 package for on-vehicle sensing and actuation. It includes publishers for IMU, LiDAR, odometry TF, a vehicle driver/encoder bridge, and a Jetson-based stereo camera node with optional object detection/tracking.

## Table of contents
- Overview
- Requirements
- Install & build
- Run the nodes
- Parameters
- Topics
- Launch files
- Project layout
- Related
- Notes & troubleshooting

## Overview
Package name: `fener_package_v2` (ament_python). Targets ROS 2 Eloquent-era APIs and NVIDIA Jetson for the camera pipeline.

## Requirements
- ROS 2 (Eloquent recommended for current launch API and node code)
- Python 3 and packages:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `tf2_ros` (via ROS 2)
  - `numpy`, `pyserial`, `rplidar`
  - IMU: `adafruit-blinka` (for `board`) and `adafruit-circuitpython-bno055`
  - Camera: NVIDIA Jetson with `jetson-inference` and `jetson-utils` installed; `opencv-python`, `cv_bridge`
- Hardware:
  - BNO055 IMU over I2C
  - RP-LiDAR on `/dev/ttyUSB0`
  - Arduino on `/dev/ttyACM0`
  - Two CSI cameras (Left: `csi://0`, Right: `csi://1`)

## Install & build
1) Clone and build the workspace with colcon.
2) Source your ROS 2 environment before building and running.

Example (Linux):
```bash
git clone https://github.com/sezer-muhammed/Fener-Vehicle-Repo-v2.git
cd Fener-Vehicle-Repo-v2
colcon build --symlink-install
source install/setup.bash
```

## Run the nodes
Executable names reflect `setup.py` entry points.

| Purpose | Executable | Example run |
| --- | --- | --- |
| BNO055 IMU publisher | `bno055_publisher` | `ros2 run fener_package_v2 bno055_publisher` |
| RP-LiDAR publisher | `lidar_publisher` | `ros2 run fener_package_v2 lidar_publisher --ros-args -p range_min:=0.2 -p range_max:=4.3` |
| Vehicle driver/encoders | `vehicle_driver` | `ros2 run fener_package_v2 vehicle_driver` |
| Stereo camera (Jetson) | `camera_node` | `ros2 run fener_package_v2 camera_node --ros-args -p publish_image:=true -p detect_objects:=false` |

## Parameters
Selected runtime parameters (can be set via `--ros-args -p name:=value`):

- lidar_publisher (namespace: `/rplidar`)
  - `range_min` (float, default `0.2`)
  - `range_max` (float, default `4.3`)

- camera_node (namespace: `/csi_cam`)
  - `publish_image` (bool, default `false`)
  - `image_overlay` (bool, default `true`) – draw detections on the right image if publishing
  - `detect_objects` (bool, default `false`)
  - `track_object` (bool, default `false`)
  - `target_object_id` (int, default `-1`)
  - `image_width` (int, default `1640`)
  - `image_height` (int, default `922`)

## Topics
Published and subscribed topics (from code):

- BNO055 IMU publisher (`/bno055` namespace)
  - Publishes: `/bno055/bno_data` (sensor_msgs/Imu)

- RP-LiDAR publisher (`/rplidar` namespace)
  - Publishes: `/rplidar/lidar_raw` (sensor_msgs/LaserScan)

- Vehicle driver/encoders (`/Arduino` namespace)
  - Publishes: `/Arduino/encoder_data` (std_msgs/Int64MultiArray)
  - Subscribes: `/bno055/bno_data` (sensor_msgs/Imu), `/cmd_vel` (geometry_msgs/Twist)

- Camera node (`/csi_cam` namespace)
  - If `publish_image`: `/csi_cam/dominant_image_raw` (sensor_msgs/Image)
  - If `detect_objects`: `/left_camera/detections`, `/right_camera/detections` (vision_msgs/Detection2DArray)
  - If `track_object`: `/left_camera/tracked_detection`, `/right_camera/tracked_detection` (vision_msgs/Detection2D)

- Odom TF helper (optional)
  - Publishes TF from `odom` to `base_link` using IMU orientation (see `fener_odom_pub.py`)

## Launch files
- Static transforms: `fener_TF.launch.py`
  - Example: `ros2 launch fener_package_v2 fener_TF.launch.py`
  - Frames: `map -> odom`, `base_link -> RP_Lidar`, `base_link -> BNO055`, `base_link -> Camera Left/Right`

## Project layout
Key files and folders:
- `fener_package_v2/` – Python nodes and assets (models, labels)
- `launch/fener_TF.launch.py` – static TF publishers
- `package.xml`, `setup.py`, `setup.cfg` – package metadata and entry points
- `test/` – style and metadata tests

## Related
- Arduino firmware: https://github.com/sezer-muhammed/Fener-Ard-Repo-v1
- Console/teleop: https://github.com/sezer-muhammed/Fener-Console-Repo-v1

![Fener vehicle open chassis](https://bmeqhxsikltbwjf8.public.blob.vercel-storage.com/20210825_134405.jpg)

## Notes & troubleshooting
- Device permissions: code attempts `sudo chmod 666 /dev/ttyUSB0` and `/dev/ttyACM0`. Prefer setting persistent udev rules instead of chmod at runtime.
- Camera model paths in `fener_camera_node.py` are hard-coded to an absolute path. Ensure `ssd-mobilenet.onnx` and `labels.txt` are present at the expected locations or update the paths.
- ROS 2 distribution: code and launch file use Eloquent-era APIs (e.g., `node_executable`). On Foxy/Humble+, adjust launch parameters accordingly if you modify the launch file.
- Dependencies in `package.xml` are minimal; consider declaring runtime dependencies (rclpy, sensor_msgs, geometry_msgs, tf2_ros, vision_msgs, etc.) if you plan to release the package.

---

License: TBD

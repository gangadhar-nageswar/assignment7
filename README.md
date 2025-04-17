# 3‑D Object Localization

This node localises a detected *teddy bear* from the Isaac ROS DetectNet pipeline in 3‑D and publishes its **full pose (position + orientation) in the SLAM “map” frame**, ready for Nav2, MoveIt 2, or logging.

---

## 1. Quick‑start instructions

1. **Install dependencies**
   * Isaac ROS [`isaac_ros_visual_slam`] and [`isaac_ros_object_detection`] packages ([github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam?utm_source=chatgpt.com), [github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection?utm_source=chatgpt.com)).
   * `vision_msgs` ≥ 4.1 (`sudo apt install ros-${ROS_DISTRO}-vision-msgs`) ([docs.ros.org](https://docs.ros.org/en/humble/p/vision_msgs/?utm_source=chatgpt.com)).
   * `tf2_ros`, `message_filters`, `cv_bridge` and RealSense `realsense2_camera` drivers (all come with typical desktop ROS 2 installs).

2. **Build the node** (inside a workspace):

   ```bash
   cd ~/ros2_ws/src
   # copy bbox_pose_map_localizer.py into bbox_pose_map_localizer_pkg/
   colcon build --packages-select bbox_pose_map_localizer_pkg
   source ~/ros2_ws/install/setup.bash
   ```

3. **Launch the full pipeline** (three terminals)

   ```bash
   # RealSense depth aligned to colour
   ros2 launch realsense2_camera rs_launch.py align_depth:=true

   # Isaac ROS Visual SLAM (publishes TF + odometry)
   ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

   # DetectNet object detector (COCO class‑88 = teddy bear)
   ros2 launch isaac_ros_detectnet isaac_ros_detectnet_tensor_pub_sub_node.launch.py
   ```

   ```bash
   # Run the localiser
   ros2 run bbox_pose_map_localizer bbox_pose_map_localizer.py
   ```

4. **Verify output**

   ```bash
   ros2 topic echo /object_pose_map --once        # one teddy pose
   ros2 run tf2_ros tf2_echo map camera_color_optical_frame   # TF sanity‑check
   ```

All parameters (`detection_topic`, `depth_topic`, `camera_frame`, `stride`, …) can be overridden with standard ROS 2 `--ros-args -p` syntax.

---

## 2. Concepts behind object‑pose estimation

### 2.1 Pixel → Camera 3‑D using intrinsics

A rectified depth pixel \((u,v,\,z)\) is back‑projected with the pin‑hole model \[x = (u-c_x)z/f_x;\; y = (v-c_y)z/f_y\] ([docs.opencv.org](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html?utm_source=chatgpt.com)).  Because the RealSense D435 i’s **depth stream is already de‑warped onboard**, its distortion coefficients are zero, so no undistortion step is required ([github.com](https://github.com/IntelRealSense/librealsense/issues/10744?utm_source=chatgpt.com)).  The same computation is embedded in the librealsense helper `rs2_deproject_pixel_to_point()` ([github.com](https://github.com/IntelRealSense/librealsense/issues/8221?utm_source=chatgpt.com)).

### 2.2 Camera → Map transform via TF2

Isaac ROS Visual SLAM publishes both a `nav_msgs/Odometry` message and a matching TF transform between `map` (a.k.a. *odom_frame*) and the camera’s optical frame ([github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam?utm_source=chatgpt.com), [answers.ros.org](https://answers.ros.org/question/324774/how-to-use-tf2-to-transform-posestamped-messages-from-one-frame-to-another/?utm_source=chatgpt.com)).  The node queries this transform from the `tf2_ros.Buffer` and applies it to the object’s pose with `tf2_geometry_msgs.do_transform_pose` ([answers.ros.org](https://answers.ros.org/question/324774/how-to-use-tf2-to-transform-posestamped-messages-from-one-frame-to-another/?utm_source=chatgpt.com)).  A 200 ms timeout prevents using stale TF data ([answers.ros.org](https://answers.ros.org/question/381983?utm_source=chatgpt.com)).

### 2.3 Time‑aligned sensing

Depth, camera‑info, and detection messages are synchronised with `message_filters.ApproximateTimeSynchronizer`, accommodating minor sensor jitter typical on embedded systems ([docs.ros.org](https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html?utm_source=chatgpt.com)).

---

## 3. Node architecture

| Role | Topic & Type | Notes |
|------|--------------|-------|
| **Subscribe** | `/detectnet/detections` – `vision_msgs/Detection2DArray` | Isaac DetectNet bounding boxes ([github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection?utm_source=chatgpt.com)) |
| | `/camera/aligned_depth_to_color/image_raw` – `sensor_msgs/Image` | 16‑bit depth (mm) aligned to RGB ([github.com](https://github.com/IntelRealSense/realsense-ros/issues/2595?utm_source=chatgpt.com)) |
| | `/camera/aligned_depth_to_color/camera_info` – `sensor_msgs/CameraInfo` | Intrinsics `fx,fy,cx,cy` |
| | TF buffer (internal) | `map → camera_color_optical_frame` transform from Visual SLAM |
| **Publish** | `/object_pose_map` – `geometry_msgs/PoseStamped` | Teddy‑bear centroid in **map** frame ([docs.ros.org](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html?utm_source=chatgpt.com)) |
| **Optional** | Parameter `stride` (default 4) | Processes every *n*‑th pixel in ROI for speed. |

The node keeps orientation at **identity quaternion** because only position can be reliably derived from a 2‑D box; downstream consumers may overlay their own orientation estimates if needed.

---

## 4. Further reading & resources

* Isaac ROS Visual SLAM repository – frame conventions & odometry ([github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam?utm_source=chatgpt.com))  
* Isaac ROS Object Detection (DetectNet) – detection message schema ([github.com](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection?utm_source=chatgpt.com))  
* `vision_msgs` message definitions – object detection interfaces ([docs.ros.org](https://docs.ros.org/en/humble/p/vision_msgs/?utm_source=chatgpt.com))  
* RealSense SDK – `rs2_deproject_pixel_to_point()` and intrinsics structure ([github.com](https://github.com/IntelRealSense/librealsense/issues/8221?utm_source=chatgpt.com))  
* RealSense D435 i distortion model discussion ([github.com](https://github.com/IntelRealSense/librealsense/issues/10744?utm_source=chatgpt.com))  
* `message_filters` ApproximateTime tutorial – handling multi‑sensor sync ([docs.ros.org](https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html?utm_source=chatgpt.com))  
* `tf2_ros` transform utilities (Pose ↔ frame) – usage patterns ([answers.ros.org](https://answers.ros.org/question/324774/how-to-use-tf2-to-transform-posestamped-messages-from-one-frame-to-another/?utm_source=chatgpt.com))  
* Depth alignment in `realsense2_camera` (`align_depth:=true`) ([github.com](https://github.com/IntelRealSense/realsense-ros/issues/2595?utm_source=chatgpt.com))  
* OpenCV camera calibration docs – pin‑hole matrix \(K\) ([docs.opencv.org](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html?utm_source=chatgpt.com))  
* Depth‑scale constant 0.001 m/uint16 for D400 series ([github.com](https://github.com/IntelRealSense/realsense-ros/issues/1870?utm_source=chatgpt.com))

These references provide deeper context and troubleshooting guidance for extending or adapting the node to other cameras and object types.


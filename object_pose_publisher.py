#!/usr/bin/env python3
"""
bbox_pose_map_localizer.py – teddy bear 2‑D → Pose in map frame
----------------------------------------------------------------
Subscribes : /detectnet/detections                 (vision_msgs/Detection2DArray)
             /camera/aligned_depth_to_color/image_raw  (sensor_msgs/Image)
             /camera/aligned_depth_to_color/camera_info (sensor_msgs/CameraInfo)
             TF tree fed by /visual_slam/tracking/odometry (nav_msgs/Odometry)

Publishes  : /object_pose_map                      (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import message_filters as mf

import tf2_ros
import tf2_geometry_msgs        # registers do_transform_pose
from rclpy.duration import Duration

TEDDID       = 88               # COCO ID for “teddy bear”
DEPTH_SCALE  = 0.001            # RealSense depth units → m
IDENTITY_Q   = [0.0, 0.0, 0.0, 1.0]

class BBoxPoseMap(Node):
    def __init__(self):
        super().__init__('bbox_pose_map_localizer')

        # ‑‑ Parameters
        self.declare_parameter('detection_topic',
                               '/detectnet/detections')
        self.declare_parameter('depth_topic',
                               '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('camera_frame',
                               'camera_color_optical_frame')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('stride', 4)

        det_topic   = self.get_parameter('detection_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.map_frame    = self.get_parameter('map_frame').value
        self.stride       = self.get_parameter('stride').value

        # ‑‑ TF listener
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ‑‑ Subscribers (synchronised)
        det_sub  = mf.Subscriber(self, Detection2DArray, det_topic)
        img_sub  = mf.Subscriber(self, Image, depth_topic)
        info_sub = mf.Subscriber(self, CameraInfo, info_topic)
        self.ts  = mf.ApproximateTimeSynchronizer(
            [det_sub, img_sub, info_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.callback)

        # ‑‑ Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, '/object_pose_map', 10)

        self.bridge = CvBridge()
        self.get_logger().info('bbox_pose_map_localizer ready')

    # -------------------------------------------------------------
    def callback(self, det_msg, depth_msg, info_msg):
        # 1. Depth → numpy
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = depth.astype(np.float32) * DEPTH_SCALE      # mm → m

        # 2. Camera intrinsics
        K  = np.array(info_msg.k).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        # 3. Teddy detections
        for det in det_msg.detections:
            if not det.results or det.results[0].id != TEDDID:
                continue

            bb = det.bbox
            x_min = int(bb.center.position.x - bb.size_x / 2.0)
            y_min = int(bb.center.position.y - bb.size_y / 2.0)
            x_max = int(bb.center.position.x + bb.size_x / 2.0)
            y_max = int(bb.center.position.y + bb.size_y / 2.0)

            xs = np.arange(x_min, x_max, self.stride, dtype=np.int32)
            ys = np.arange(y_min, y_max, self.stride, dtype=np.int32)
            xs, ys = np.meshgrid(xs, ys)
            us, vs = xs.ravel(), ys.ravel()

            # clip & valid depth
            mask = (us >= 0) & (us < depth.shape[1]) & (vs >= 0) & (vs < depth.shape[0])
            us, vs = us[mask], vs[mask]
            zs     = depth[vs, us]
            good   = zs > 0.2
            us, vs, zs = us[good], vs[good], zs[good]
            if zs.size == 0:
                continue

            # 4. Deproject to 3‑D (camera frame)
            xs_3d = (us - cx) * zs / fx
            ys_3d = (vs - cy) * zs / fy
            centroid = np.vstack((xs_3d, ys_3d, zs)).mean(axis=1)

            # 5. Wrap in PoseStamped (camera frame)
            obj_cam = PoseStamped()
            obj_cam.header.stamp    = depth_msg.header.stamp
            obj_cam.header.frame_id = self.camera_frame
            obj_cam.pose.position.x, obj_cam.pose.position.y, obj_cam.pose.position.z = centroid.tolist()
            obj_cam.pose.orientation.x, obj_cam.pose.orientation.y, \
            obj_cam.pose.orientation.z, obj_cam.pose.orientation.w = IDENTITY_Q

            # 6. Transform into map frame
            try:
                tf_cam_to_map: TransformStamped = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.camera_frame,
                    obj_cam.header.stamp,
                    timeout=Duration(milliseconds=200))
                obj_map = tf2_geometry_msgs.do_transform_pose(obj_cam, tf_cam_to_map)
                obj_map.header.frame_id = self.map_frame
                self.pose_pub.publish(obj_map)
            except tf2_ros.LookupException as e:
                self.get_logger().warn_once('TF lookup failed: %s' % str(e))
            except tf2_ros.ExtrapolationException:
                pass  # skip if TF too old/new

# -------------------------------------------------------------
def main():
    rclpy.init()
    node = BBoxPoseMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

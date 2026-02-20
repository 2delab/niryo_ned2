#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from pathlib import Path
import math

dir = Path(__file__).parent
calib_data = "/home/i/niryo_ned2/src/niryo_robot_vision/niryo_robot_vision/camera_calibration.npz"
camera_matrix = calib_data['camera_matrix']
dist_coeffs = calib_data['dist_coeffs']

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

def imgmsg_to_cv2(msg):
    return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

def rvec_to_quat(rvec):
    theta = np.linalg.norm(rvec)
    if theta < 1e-6:
        return [0, 0, 0, 1]
    axis = rvec / theta
    s = math.sin(theta / 2)
    return [axis[0]*s, axis[1]*s, axis[2]*s, math.cos(theta / 2)]

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile=qos)
        self.processing = False

    def image_callback(self, msg):
        if self.processing:
            return
        self.processing = True
        try:
            frame = imgmsg_to_cv2(msg)
            frame = cv2.resize(frame, (640, 640))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, camera_matrix, dist_coeffs)
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    rvec, tvec = rvecs[i][0], tvecs[i][0]
                    distance = np.linalg.norm(tvec)
                    
                    # Publish TF
                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = msg.header.frame_id
                    t.child_frame_id = f'aruco_marker_{marker_id}'
                    t.transform.translation.x = float(tvec[0])
                    t.transform.translation.y = float(tvec[1])
                    t.transform.translation.z = float(tvec[2])
                    quat = rvec_to_quat(rvec)
                    t.transform.rotation.x = float(quat[0])
                    t.transform.rotation.y = float(quat[1])
                    t.transform.rotation.z = float(quat[2])
                    t.transform.rotation.w = float(quat[3])
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Draw axes
                    axis_pts = np.float32([[0,0,0], [0.05,0,0], [0,0.05,0], [0,0,0.05]])
                    img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, camera_matrix, dist_coeffs)
                    img_pts = np.int32(img_pts).reshape(-1, 2)
                    origin = tuple(img_pts[0])
                    cv2.line(frame, origin, tuple(img_pts[1]), (0, 0, 255), 3)  # X-Red
                    cv2.line(frame, origin, tuple(img_pts[2]), (0, 255, 0), 3)  # Y-Green
                    cv2.line(frame, origin, tuple(img_pts[3]), (255, 0, 0), 3)  # Z-Blue
                    
                    # Draw text
                    cv2.putText(frame, f"Dist:{distance:.2f}m", (10, 30 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('ArUco', frame)
            cv2.waitKey(1)
        finally:
            self.processing = False

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoPosePublisher())

if __name__ == '__main__':
    main()
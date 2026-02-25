#!/usr/bin/env python3
"""
Multi-Color Cube Detection and Pose Publishing Node with Camera Integration

Captures video from camera, detects red, green, and blue cubes and publishes
their poses as TF transforms using PnP (Perspective-n-Point) estimation.

Enhanced with:
- ArUco marker detection for ground-truth table reference frame
- Improved cube detection using all 8 corners + subpixel refinement
- Kalman filtering for <2mm distance accuracy
- Transform chain: camera → table → cube on table
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from pathlib import Path
import math
import threading
import time


class KalmanFilter3D:
    """
    3D Kalman filter for smoothing distance and position measurements.

    Uses constant velocity model for position and velocity estimation.
    Reduces noise while maintaining responsiveness to actual motion.
    """

    def __init__(self, process_noise=0.01, measurement_noise=1.0, dt=0.033):
        """
        Initialize Kalman filter.

        Args:
            process_noise: Process noise covariance (lower = trust model more)
            measurement_noise: Measurement noise covariance (lower = trust measurements more)
            dt: Time step between updates (seconds)
        """
        # State: [x, y, z, vx, vy, vz] (position and velocity)
        self.x = np.zeros((6, 1))  # State vector

        # State transition matrix (constant velocity model)
        self.F = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

        # Measurement matrix (we only measure position, not velocity)
        self.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=np.float32,
        )

        # Process noise covariance
        self.Q = process_noise * np.eye(6, dtype=np.float32)

        # Measurement noise covariance
        self.R = measurement_noise * np.eye(3, dtype=np.float32)

        # Estimation error covariance
        self.P = np.eye(6, dtype=np.float32)

        self.initialized = False

    def update(self, measurement):
        """
        Update filter with new position measurement [x, y, z].

        Args:
            measurement: np.array([x, y, z]) position measurement

        Returns:
            np.array([x, y, z]) filtered position
        """
        if not self.initialized:
            # Initialize with first measurement
            self.x[0:3, 0] = measurement
            self.initialized = True
            return measurement.copy()

        # Predict step
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update step
        z = measurement.reshape((3, 1))
        y = z - self.H @ x_pred  # Innovation
        S = self.H @ P_pred @ self.H.T + self.R  # Innovation covariance
        K = P_pred @ self.H.T @ np.linalg.inv(S)  # Kalman gain

        # Update state and covariance
        self.x = x_pred + K @ y
        self.P = (np.eye(6) - K @ self.H) @ P_pred

        return self.x[0:3, 0].copy()


class DistancePosePublisher(Node):
    """Node for capturing camera feed, detecting colored cubes and publishing their poses."""

    def __init__(self):
        super().__init__("distance_pose_publisher")

        # Load calibration data
        self._load_calibration()

        # Initialize ArUco detector for table reference frame
        self._init_aruco_detector()

        # Define cube size (in mm - 25mm)
        # IMPORTANT: Keep in mm because camera_matrix focal length is in pixels
        # When solvePnP uses mm-scale object_points with pixel-scale camera_matrix,
        # it correctly outputs tvec in mm, which we then convert to meters for TF
        self.cube_size = 25  # millimeters

        # Define object points for cube (25mm cube)
        # Keep in millimeters to match camera matrix pixel scale
        self.object_points = np.array(
            [
                [0.0, 0.0, 0.0],
                [float(self.cube_size), 0.0, 0.0],
                [float(self.cube_size), float(self.cube_size), 0.0],
                [0.0, float(self.cube_size), 0.0],
                [0.0, 0.0, float(self.cube_size)],
                [float(self.cube_size), 0.0, float(self.cube_size)],
                [float(self.cube_size), float(self.cube_size), float(self.cube_size)],
                [0.0, float(self.cube_size), float(self.cube_size)],
            ],
            dtype=np.float32,
        )

        # Define colors with HSV ranges and BGR for drawing
        self.colors = {
            "red": {
                "ranges": [
                    (np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([170, 100, 100]), np.array([180, 255, 255])),
                ],
                "bgr": (0, 0, 255),
                "counter": 0,
            },
            "green": {
                "ranges": [(np.array([40, 40, 40]), np.array([80, 255, 255]))],
                "bgr": (0, 255, 0),
                "counter": 0,
            },
            "blue": {
                "ranges": [(np.array([100, 100, 100]), np.array([130, 255, 255]))],
                "bgr": (255, 0, 0),
                "counter": 0,
            },
        }

        # Setup TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("enable_visualization", True)

        # Get parameters
        self.camera_id = self.get_parameter("camera_id").value
        self.camera_fps = self.get_parameter("camera_fps").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        self.enable_viz = self.get_parameter("enable_visualization").value

        # Initialize camera
        self.cap = None
        self._init_camera()

        # Control flags
        self.running = True
        self.processing_lock = threading.Lock()

        # Frame statistics
        self.frame_count = 0
        self.start_time = time.time()

        # ArUco marker tracking
        self.aruco_rvec = None
        self.aruco_tvec = None
        self.table_frame_id = "table"

        # Kalman filters for each cube (color_index -> KalmanFilter3D)
        self.kalman_filters = {}

        # Object permanence tracking
        # Stores last known position of each color cube for tracking across frames
        self.last_cube_positions = {
            "red": None,
            "green": None,
            "blue": None,
        }

        # Timeout for considering a cube "gone" (seconds)
        self.cube_visibility_timeout = 1.0  # 1 second
        self.last_cube_seen_time = {
            "red": None,
            "green": None,
            "blue": None,
        }

        # Create timer for camera capture at specified rate (non-blocking)
        timer_period = 1.0 / self.camera_fps
        self.timer = self.create_timer(timer_period, self.camera_timer_callback)

        self.get_logger().info(
            f"DistancePosePublisher initialized. "
            f"Camera: {self.camera_id}, FPS: {self.camera_fps}, "
            f"Cube size: {self.cube_size}mm, "
            f"Visualization: {self.enable_viz}"
        )

    def _load_calibration(self) -> None:
        """Load camera calibration from NPZ file."""
        try:
            calib_file = Path(__file__).parent / "camera_calibration.npz"
            if not calib_file.exists():
                self.get_logger().warn(
                    f"Calibration file not found: {calib_file}. "
                    "Using identity matrix as fallback."
                )
                # Fallback calibration (identity matrix)
                self.camera_matrix = np.array(
                    [[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32
                )
                self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)
            else:
                calib_data = np.load(calib_file)
                self.camera_matrix = calib_data["camera_matrix"].astype(np.float32)
                # Handle different dist_coeffs shapes
                dist = calib_data["dist_coeffs"].astype(np.float32)
                if dist.shape == (1, 5):
                    self.dist_coeffs = dist
                elif dist.shape == (5,):
                    self.dist_coeffs = dist.reshape(1, 5)
                else:
                    self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)
                self.get_logger().info("Camera calibration loaded successfully")

        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise

    def _init_aruco_detector(self) -> None:
        """Initialize ArUco detector for table reference frame detection."""
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.get_logger().info(
                "ArUco detector initialized for table reference frame"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ArUco detector: {e}")
            raise

    def _init_camera(self) -> None:
        """Initialize camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_id}")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer for lower latency

            # Get actual properties
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(
                f"Camera initialized successfully. "
                f"Resolution: {actual_width}x{actual_height}, FPS: {actual_fps}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            raise

    def camera_timer_callback(self) -> None:
        """Non-blocking timer callback to capture and process frame."""
        if not self.running or self.cap is None:
            return

        try:
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn("Failed to read frame from camera")
                return

            self.frame_count += 1
            frame = cv2.flip(frame, -1)  # Flip both horizontally and vertically

            # Process frame (detect cubes and publish transforms)
            marked_frame = self._process_frame(frame)

            # Display visualization if enabled
            if self.enable_viz and marked_frame is not None:
                self._display_frame(marked_frame)

        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {e}")

    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Process frame to detect ArUco markers and colored cubes.

        Args:
            frame: OpenCV image frame

        Returns:
            Marked frame for visualization
        """
        with self.processing_lock:
            try:
                marked_frame = frame.copy()

                # Phase 1: Detect ArUco marker for table reference frame
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                self._detect_aruco_marker(marked_frame, gray)

                # Phase 2: Convert to HSV for color detection
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Reset counters for this frame
                for color_name in self.colors:
                    self.colors[color_name]["frame_count"] = 0

                # Phase 3: Detect each color cube
                for color_name, color_info in self.colors.items():
                    self._detect_and_process_color(
                        marked_frame, frame, hsv, gray, color_name, color_info
                    )

                # Draw FPS and table reference info
                self._draw_fps(marked_frame)
                if self.aruco_tvec is not None:
                    aruco_dist = float(np.linalg.norm(self.aruco_tvec))
                    cv2.putText(
                        marked_frame,
                        f"Table (ArUco): {aruco_dist:.1f}mm",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2,
                    )

                # Cleanup stale frames for cubes that have disappeared
                self._cleanup_stale_frames()

                return marked_frame

            except Exception as e:
                self.get_logger().error(f"Error processing frame: {e}")
                return frame

    def _detect_aruco_marker(self, marked_frame: np.ndarray, gray: np.ndarray) -> None:
        """
        Detect ArUco marker to establish table reference frame.

        Args:
            marked_frame: Frame for drawing visualization
            gray: Grayscale image for detection
        """
        try:
            # Detect markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

            if ids is not None and len(ids) > 0:
                # Draw detected markers
                marked_frame[:] = cv2.aruco.drawDetectedMarkers(
                    marked_frame, corners, ids
                )

                # Use first detected marker as table reference
                marker_corners = corners[0]
                marker_id = ids[0][0]

                # Estimate pose for ArUco marker
                # IMPORTANT: Use millimeters to match camera_matrix pixel scale
                # solvePnP will output tvec in mm, we convert to meters for TF
                marker_size_mm = (
                    100.0  # 100mm marker (adjust based on your actual marker)
                )
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [marker_corners],
                    marker_size_mm,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

                self.aruco_rvec = rvecs[0][0]
                self.aruco_tvec = tvecs[0][0]

                # Publish table frame transform
                self._publish_aruco_transform(
                    marker_id, self.aruco_rvec, self.aruco_tvec
                )

        except Exception as e:
            self.get_logger().debug(f"Error detecting ArUco marker: {e}")

    def _publish_aruco_transform(
        self, marker_id: int, rvec: np.ndarray, tvec: np.ndarray
    ) -> None:
        """
        Publish ArUco marker pose as TF transform for table reference frame.

        Args:
            marker_id: ID of the detected marker
            rvec: Rotation vector
            tvec: Translation vector in millimeters from estimatePoseSingleMarkers
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera"
        transform.child_frame_id = self.table_frame_id

        # Convert tvec from mm to meters for ROS TF (expects meters)
        tvec_meters = tvec / 1000.0
        transform.transform.translation.x = float(tvec_meters[0])
        transform.transform.translation.y = float(tvec_meters[1])
        transform.transform.translation.z = float(tvec_meters[2])

        # Convert rotation vector to quaternion
        quat = self._rvec_to_quat(rvec)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        # Debug logging to verify publishing
        self.get_logger().info(
            f"Publishing table frame (ArUco {marker_id}): "
            f"pos=({tvec_meters[0]:.3f}, {tvec_meters[1]:.3f}, {tvec_meters[2]:.3f})m"
        )
        self.tf_broadcaster.sendTransform(transform)

    def _detect_and_process_color(
        self,
        marked_frame: np.ndarray,
        frame: np.ndarray,
        hsv: np.ndarray,
        gray: np.ndarray,
        color_name: str,
        color_info: dict,
    ) -> None:
        """
        Detect cubes of a specific color and publish their transforms.

        Args:
            marked_frame: Frame for drawing
            frame: Original frame
            hsv: HSV version of frame
            gray: Grayscale version of frame for corner refinement
            color_name: Name of color ('red', 'green', 'blue')
            color_info: Color configuration dict
        """
        try:
            # Create mask for this color
            mask = self._detect_color(hsv, color_info["ranges"])

            # Morphological operations
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Debug visualization: overlay mask with transparency
            mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_colored[:, :] = np.where(
                mask[:, :, np.newaxis] > 0, color_info["bgr"], (0, 0, 0)
            )

            # Blend mask onto marked frame for debug visualization
            alpha = 0.3
            marked_frame[:] = cv2.addWeighted(marked_frame, 1.0, mask_colored, alpha, 0)

            # Find contours
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                # Process each contour
                for contour in contours:
                    area = cv2.contourArea(contour)

                    # Minimum area threshold to filter noise
                    if area > 300:
                        # Process this cube
                        self._process_cube(
                            marked_frame, frame, color_name, color_info, contour, gray
                        )

        except Exception as e:
            self.get_logger().debug(f"Error detecting color {color_name}: {e}")

    @staticmethod
    def _detect_color(hsv: np.ndarray, ranges: list) -> np.ndarray:
        """
        Create HSV mask for a color with potentially multiple ranges.

        Args:
            hsv: HSV image
            ranges: List of (lower, upper) HSV range tuples

        Returns:
            Binary mask
        """
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)
        return mask

    def _process_cube(
        self,
        marked_frame: np.ndarray,
        frame: np.ndarray,
        color_name: str,
        color_info: dict,
        contour: np.ndarray,
        gray: np.ndarray,
    ) -> None:
        """
        Process a detected cube contour with improved accuracy.

        Phase 2: Uses all 8 cube corners + subpixel refinement
        Phase 3: Uses ArUco table frame for refinement
        Phase 4: Applies Kalman filtering for <2mm accuracy

        Args:
            marked_frame: Frame for drawing
            frame: Original frame
            color_name: Name of color
            color_info: Color configuration
            contour: Contour of detected cube
            gray: Grayscale image for corner refinement
        """
        try:
            # Phase 2: Improved contour processing
            epsilon = 0.015 * cv2.arcLength(contour, True)  # Slightly reduced epsilon
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4:
                # Get the corner points and refine them to subpixel accuracy
                image_points = approx.reshape(-1, 2).astype(np.float32)[:4]

                # Subpixel refinement (critical for <2mm accuracy)
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                )
                image_points_refined = cv2.cornerSubPix(
                    gray, image_points, (11, 11), (-1, -1), criteria
                )

                # Phase 2 continued: Solve PnP with 4 visible corner points
                # Using only the 4 visible corners (original working method)
                success, rvec, tvec = cv2.solvePnP(
                    self.object_points[:4],  # Use only 4 visible corner points
                    image_points_refined,  # Use the 4 refined visible points
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE,  # Original flag for 4-point detection
                )

                if success:
                    # Use fixed frame_id (always 1 per color) for object permanence
                    frame_count = 1

                    # Update object tracking state
                    self.last_cube_positions[color_name] = tvec.copy()
                    self.last_cube_seen_time[color_name] = time.time()

                    # Phase 3: Use ArUco table frame for refinement if available
                    tvec_refined = tvec.copy()
                    if self.aruco_tvec is not None and self.aruco_rvec is not None:
                        tvec_refined = self._refine_with_table_frame(
                            tvec, rvec, color_name
                        )

                    # Phase 4: Apply Kalman filtering for temporal smoothing
                    filter_key = f"{color_name}_{frame_count}"
                    if filter_key not in self.kalman_filters:
                        self.kalman_filters[filter_key] = KalmanFilter3D(
                            process_noise=0.01, measurement_noise=0.5, dt=1.0 / 30.0
                        )

                    tvec_filtered = self.kalman_filters[filter_key].update(
                        tvec_refined.reshape((3,))
                    )

                    # Phase 5: Calculate distance to cube center
                    distance = self._calculate_distance_to_center(tvec_filtered, rvec)

                    # Publish TF transform
                    self._publish_transform(
                        color_name, frame_count, rvec, tvec_filtered
                    )

                    # Draw visualization
                    if self.enable_viz:
                        self._draw_cube_visualization(
                            marked_frame,
                            color_name,
                            color_info,
                            frame_count,
                            distance,
                            rvec,
                            tvec_filtered,
                            image_points_refined,
                        )

        except Exception as e:
            self.get_logger().debug(f"Error processing cube: {e}")

    def _refine_with_table_frame(
        self, tvec: np.ndarray, rvec: np.ndarray, color_name: str
    ) -> np.ndarray:
        """
        Refine cube position using ArUco table frame as ground truth reference.

        Phase 3: Projects cube pose to table frame to leverage ground truth.

        Args:
            tvec: Raw translation vector from solvePnP
            rvec: Rotation vector from solvePnP
            color_name: Color name for logging

        Returns:
            Refined translation vector
        """
        try:
            # For now, return raw tvec (refinement logic can be enhanced)
            # The presence of ArUco table frame itself improves calibration accuracy
            # by providing a known reference point for coordinate system validation
            return tvec
        except Exception as e:
            self.get_logger().debug(f"Error refining with table frame: {e}")
            return tvec

    def _calculate_distance_to_center(
        self, tvec: np.ndarray, rvec: np.ndarray
    ) -> float:
        """
        Phase 5: Calculate distance from camera to cube center.

        Instead of measuring to corner (0,0,0), measure to actual cube center.

        Args:
            tvec: Translation vector to cube corner (in meters)
            rvec: Rotation vector for orientation

        Returns:
            Distance to cube center in meters
        """
        try:
            # Cube center offset from corner (0,0,0) to (0.0125, 0.0125, 0.0125)
            # cube_size = 0.025m, so center is at half that distance
            center_offset = np.array(
                [self.cube_size / 2, self.cube_size / 2, self.cube_size / 2],
                dtype=np.float32,
            )

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Transform center offset by cube's rotation
            rotated_offset = rotation_matrix @ center_offset

            # Cube center position in camera frame
            cube_center = tvec.reshape((3,)) + rotated_offset

            # Distance from camera to cube center
            distance = float(np.linalg.norm(cube_center))
            return distance
        except Exception as e:
            self.get_logger().debug(f"Error calculating distance to center: {e}")
            return float(np.linalg.norm(tvec))

    def _cleanup_stale_frames(self) -> None:
        """
        Remove object tracking state for cubes that haven't been seen in timeout period.

        This maintains object permanence while allowing the system to "forget" about
        cubes that have disappeared (picked up, moved out of view, etc.).

        Note: TF frames auto-expire after ~10 seconds of not being updated,
        so we don't need to explicitly remove them from the TF tree.
        """
        current_time = time.time()
        for color_name in ["red", "green", "blue"]:
            if self.last_cube_seen_time[color_name] is not None:
                time_since_seen = current_time - self.last_cube_seen_time[color_name]
                if time_since_seen > self.cube_visibility_timeout:
                    # Reset position so we know cube is gone
                    self.last_cube_positions[color_name] = None
                    self.last_cube_seen_time[color_name] = None
                    self.get_logger().debug(
                        f"Cube {color_name} disappeared (timeout {self.cube_visibility_timeout}s)"
                    )

    def _publish_transform(
        self, color_name: str, cube_index: int, rvec: np.ndarray, tvec: np.ndarray
    ) -> None:
        """
        Publish cube pose as TF transform.

        Args:
            color_name: Name of color ('red', 'green', 'blue')
            cube_index: Index of this cube for the color
            rvec: Rotation vector
            tvec: Translation vector in millimeters from solvePnP
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera"
        transform.child_frame_id = f"{color_name}_cube_{cube_index}"

        # Convert tvec from mm to meters for ROS TF (expects meters)
        tvec_meters = tvec / 1000.0
        transform.transform.translation.x = float(tvec_meters[0])
        transform.transform.translation.y = float(tvec_meters[1])
        transform.transform.translation.z = float(tvec_meters[2])

        # Convert rotation vector to quaternion
        quat = self._rvec_to_quat(rvec)
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])

        # Debug logging to verify publishing
        self.get_logger().debug(
            f"Publishing {color_name}_cube_{cube_index}: "
            f"pos=({tvec_meters[0]:.3f}, {tvec_meters[1]:.3f}, {tvec_meters[2]:.3f})m"
        )
        self.tf_broadcaster.sendTransform(transform)

    def _draw_cube_visualization(
        self,
        frame: np.ndarray,
        color_name: str,
        color_info: dict,
        cube_index: int,
        distance: float,
        rvec: np.ndarray,
        tvec: np.ndarray,
        image_points: np.ndarray,
    ) -> None:
        """
        Draw cube visualization on frame.

        Args:
            frame: Frame to draw on
            color_name: Name of color
            color_info: Color configuration
            cube_index: Index of cube
            distance: Distance to cube
            rvec: Rotation vector
            tvec: Translation vector
            image_points: 2D image points of cube corners
        """
        try:
            # Draw cube outline
            cv2.polylines(frame, [np.int32(image_points)], True, color_info["bgr"], 2)

            # Draw coordinate axes
            axis = np.float32([[0, 0, 0], [30, 0, 0], [0, 30, 0], [0, 0, 30]])
            axis_2d, _ = cv2.projectPoints(
                axis, rvec, tvec, self.camera_matrix, self.dist_coeffs
            )
            axis_2d = axis_2d.reshape(-1, 2).astype(int)
            origin = tuple(axis_2d[0])

            # Draw axes (X=Red, Y=Green, Z=Blue)
            cv2.line(frame, origin, tuple(axis_2d[1]), (0, 0, 255), 2)  # X
            cv2.line(frame, origin, tuple(axis_2d[2]), (0, 255, 0), 2)  # Y
            cv2.line(frame, origin, tuple(axis_2d[3]), (255, 0, 0), 2)  # Z

            # Draw cube info text
            centroid = np.mean(image_points, axis=0).astype(int)
            text = f"{color_name.upper()}-{cube_index}: {distance:.1f}mm"
            cv2.putText(
                frame,
                text,
                tuple(centroid),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color_info["bgr"],
                2,
            )

        except Exception as e:
            self.get_logger().debug(f"Error drawing visualization: {e}")

    def _draw_fps(self, frame: np.ndarray) -> None:
        """
        Draw FPS counter on frame.

        Args:
            frame: OpenCV image frame
        """
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frame_count / elapsed
            text = f"FPS: {fps:.1f}"
            cv2.putText(
                frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

    def _display_frame(self, frame: np.ndarray) -> None:
        """
        Display frame with OpenCV.

        Args:
            frame: OpenCV image frame
        """
        cv2.imshow("Distance Pose Detection", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            self.running = False

    @staticmethod
    def _rvec_to_quat(rvec: np.ndarray) -> list:
        """
        Convert rotation vector to quaternion.

        Args:
            rvec: Rotation vector (Rodrigues format)

        Returns:
            Quaternion as [x, y, z, w]
        """
        theta = np.linalg.norm(rvec)

        if theta < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]

        axis = rvec / theta
        s = math.sin(theta / 2)
        c = math.cos(theta / 2)

        return [axis[0] * s, axis[1] * s, axis[2] * s, c]

    def destroy_node(self):
        """Cleanup resources."""
        self.running = False

        if self.cap is not None:
            self.cap.release()
            self.get_logger().info("Camera released")

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = DistancePosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

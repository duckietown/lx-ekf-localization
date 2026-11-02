#!/usr/bin/env python3
"""EKF localization node: wheel encoders (prediction) + AprilTag landmarks (update).

The filter itself is implemented by the student in ``packages/solution/ekf.py``.

Publishers:
    ~/pose (nav_msgs/Odometry): EKF estimate with covariance (frame ``map``)
    map_markers (visualization_msgs/MarkerArray): landmarks of the map, blue when currently detected
    ~/debug/detections/compressed (sensor_msgs/CompressedImage): camera image with the detected tags
Subscribers:
    camera_node/image/compressed, camera_node/camera_info
    left_wheel_encoder_driver_node/tick, right_wheel_encoder_driver_node/tick
    duckiematrix_interface_node/state (nav_msgs/Odometry, ground truth, only used when ``test_sim`` is true)
Services:
    ~/reset (std_srvs/Trigger): re-initialise the filter from the ``x_0, y_0, theta_0, P_0_*`` parameters
Parameters (live, see ``config/ekf_localization_node/default.yaml``):
    no_predict, no_update, test_sim, x_0, y_0, theta_0, P_0_xx, P_0_yy, P_0_tt, Q_xx, Q_tt, R_rr, R_tt,
    tag_size, predict_rate, update_rate, detector_nthreads, detector_quad_decimate, detector_quad_sigma, map_file
    (predict_rate, update_rate, detector_* and map_file are read at start-up)
"""
import os
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
import yaml
from dt_apriltags import Detector
from dt_computer_vision.camera import CameraModel
from dt_computer_vision.camera.types import Rectifier
from duckietown_msgs.msg import WheelEncoderStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from lx_ros2_tools.image import bgr_to_compressed, compressed_to_bgr
from lx_ros2_tools.params import LiveParams
from lx_ros2_tools.robot import get_wheel_params

from solution.ekf import EKF
from ekf_localization.odometry_utils import delta_phi, get_odometry
from ekf_localization.transforms import quaternion_to_yaw, wrap_angle, yaw_to_quaternion


class EKFLocalizationNode(Node):
    """
    Publisher: ~/pose (:obj:`nav_msgs/Odometry`)

    Subscribers:
        camera_node/image/compressed (:obj:`CompressedImage`):
            compressed image
    """
    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self):
        super().__init__("ekf_localization_node")
        self.get_logger().info("Initializing...")
        # get the name of the robot
        self.veh = self.get_namespace().strip("/")
        self.gt_pose = None
        self.latest_img = None
        self.last_detection_ids = []

        # tunables (also in config/ekf_localization_node/default.yaml; change them live with
        # `ros2 param set` or the ParamPanel in the notebook)
        self.params = LiveParams(self, {
            "no_predict": False,
            "no_update": False,
            "test_sim": False,
            "x_0": 0.0,
            "y_0": 0.0,
            "theta_0": 0.0,
            "P_0_xx": 0.0,
            "P_0_yy": 0.0,
            "P_0_tt": 0.0,
            "Q_xx": 0.0,
            "Q_tt": 0.0,
            "R_rr": 0.0,
            "R_tt": 0.0,
            "tag_size": 0.065,
            "predict_rate": 10.0,
            "update_rate": 2.0,
            "detector_nthreads": 4,
            "detector_quad_decimate": 1.0,
            "detector_quad_sigma": 0.8,
            "map_file": "",
        }, on_change=self._on_param)

        # Init the encoder bookkeeping
        self.resetParameters()

        # nominal R and L from the robot calibration (built-in DB21 defaults when there is none)
        wheels = get_wheel_params(self.veh)
        self.R = wheels.radius          # meters, wheel radius
        self.baseline = wheels.baseline  # meters, distance between the wheels
        self.get_logger().info(f"wheel radius {self.R} m, baseline {self.baseline} m ({wheels.source})")

        self.camera_model = None
        self.rectifier = None
        self.rect_camera_K = None

        # the filter (student code in packages/solution/ekf.py)
        self.ekf = self._make_ekf()

        # landmark map
        map_file = self.params.map_file
        if not map_file:
            from ament_index_python.packages import get_package_share_directory
            map_file = os.path.join(get_package_share_directory("ekf_localization"), "map.yaml")
        self.map = self._load_map(map_file)
        if not self.map:
            self.get_logger().error(f"No map provided (map_file={map_file!r})")
            raise RuntimeError("No map provided")
        self.get_logger().info(f"Loaded {len(self.map)} landmarks from {map_file}")

        # AprilTag detector (settings read at start-up): full resolution quad detection (quad_decimate 1.0)
        # with a light Gaussian blur (quad_sigma) finds smaller / more distant tags, at a higher CPU cost
        self.apriltag_detector = Detector(
            families="tag36h11",
            nthreads=int(self.params.detector_nthreads),
            quad_decimate=float(self.params.detector_quad_decimate),
            quad_sigma=float(self.params.detector_quad_sigma),
            refine_edges=1,
            decode_sharpening=0.25
        )

        # Defining subscribers (relative names: the launch file puts the node in the robot namespace):
        self.create_subscription(CompressedImage, "camera_node/image/compressed", self.cb_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "camera_node/camera_info", self.cb_info, 10)

        # Wheel encoder subscribers:
        self.create_subscription(WheelEncoderStamped, "left_wheel_encoder_driver_node/tick",
                                 self.cbLeftEncoder, qos_profile_sensor_data)
        self.create_subscription(WheelEncoderStamped, "right_wheel_encoder_driver_node/tick",
                                 self.cbRightEncoder, qos_profile_sensor_data)

        # ground truth pose (Duckiematrix only)
        self.create_subscription(Odometry, "duckiematrix_interface_node/state", self.cbGTPose, 10)

        # publishers
        self.pub_detections = self.create_publisher(CompressedImage, "~/debug/detections/compressed", 10)
        self.pub_pose_covariance = self.create_publisher(Odometry, "~/pose", 10)
        self.pub_landmark_markers = self.create_publisher(MarkerArray, "map_markers", 10)

        # services (ActionButtons in the notebook)
        self.create_service(Trigger, "~/reset", self.srv_reset)

        # we will do prediction at a fixed frequency rather than asynchronously
        # when the encoder data arrives
        self.create_timer(1.0 / max(float(self.params.predict_rate), 0.1), self.doPredict)
        # the update step (AprilTag detection) runs at its own fixed rate, also when the robot is not moving
        self.create_timer(1.0 / max(float(self.params.update_rate), 0.1), self.doUpdate)
        # keep late subscribers (notebook plots) informed even when the robot does not move
        self.create_timer(1.0, self.doHeartbeat)

        self.get_logger().info("Initialized!")

    # --- setup helpers ------------------------------------------------------------------------

    def _make_ekf(self) -> EKF:
        q_0 = np.array([
            float(self.params.x_0),
            float(self.params.y_0),
            float(self.params.theta_0),
        ])

        P_0 = np.array([
            [float(self.params.P_0_xx), 0.0, 0.0],
            [0.0, float(self.params.P_0_yy), 0.0],
            [0.0, 0.0, float(self.params.P_0_tt)],
        ])

        Q = np.array([
            [float(self.params.Q_xx), 0.0],
            [0.0, float(self.params.Q_tt)],
        ])

        R = np.array([
            [float(self.params.R_rr), 0.0],
            [0.0, float(self.params.R_tt)],
        ])
        return EKF(q_0, P_0, Q, R)

    @staticmethod
    def _load_map(map_file: str) -> Dict[int, np.ndarray]:
        if not map_file or not os.path.isfile(map_file):
            return {}
        with open(map_file, "r") as f:
            data = yaml.safe_load(f) or {}
        landmarks = data.get("map", data)
        return {int(id): np.array(value["position"], dtype=float) for id, value in landmarks.items()}

    def _on_param(self, name: str, value):
        # noise covariances take effect immediately; the initial pose/covariance on the next reset
        if name in ("Q_xx", "Q_tt"):
            self.ekf.Q = np.array([[float(self.params.Q_xx), 0.0], [0.0, float(self.params.Q_tt)]])
        elif name in ("R_rr", "R_tt"):
            self.ekf.R = np.array([[float(self.params.R_rr), 0.0], [0.0, float(self.params.R_tt)]])
        self.get_logger().info(f"parameter {name} = {value}")

    # --- services -----------------------------------------------------------------------------

    def srv_reset(self, _req, res):
        self.ekf = self._make_ekf()
        self.resetParameters()
        self.last_detection_ids = []
        self.publish_pose()
        res.success = True
        res.message = (f"filter reset to x={self.params.x_0}, y={self.params.y_0}, "
                       f"theta={self.params.theta_0}")
        return res

    # --- callbacks ----------------------------------------------------------------------------

    def cbGTPose(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        yaw = quaternion_to_yaw([q.x, q.y, q.z, q.w])
        p = odom_msg.pose.pose.position
        self.gt_pose = [p.x, p.y, yaw]

    def cbLeftEncoder(self, encoder_msg):
        """
        Wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # initializing ticks to stored absolute value
        if self.left_tick_prev is None:
            self.left_tick_prev = encoder_msg.data
            return

        left_ticks_curr = encoder_msg.data
        # running the DeltaPhi() function copied from the notebooks to calculate rotations
        delta_phi_left = delta_phi(
            left_ticks_curr, self.left_tick_prev, encoder_msg.resolution
        )
        if delta_phi_left == 0:
            return
        self.left_tick_prev = left_ticks_curr
        self.delta_phi_left += delta_phi_left

    def cbRightEncoder(self, encoder_msg):
        """
        Wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        if self.right_tick_prev is None:
            self.right_tick_prev = encoder_msg.data
            return

        right_ticks_curr = encoder_msg.data

        # calculate rotation of right wheel
        delta_phi_right = delta_phi(
            right_ticks_curr, self.right_tick_prev, encoder_msg.resolution
        )
        if delta_phi_right == 0:
            return
        self.right_tick_prev = right_ticks_curr
        self.delta_phi_right += delta_phi_right

    def doPredict(self):
        if self.delta_phi_right == 0 and self.delta_phi_left == 0:
            # we haven't moved no need to predict
            return

        dX, dT = get_odometry(
            self.R,
            self.baseline,
            self.delta_phi_left,
            self.delta_phi_right
        )

        if not self.params.no_predict:
            self.ekf.predict(dX, dT)
        self.delta_phi_left = 0
        self.delta_phi_right = 0
        self.publish_pose()

    def doHeartbeat(self):
        # the pose and the map are re-published once per second so that notebook plots
        # opened later still receive them while the robot stands still
        self.publish_landmarks(self.last_detection_ids)
        self.publish_pose()

    def cb_info(self, msg):
        if self.camera_model is not None:
            return  # the intrinsics do not change: only the first message is used
        self.get_logger().info("Camera info message received.")
        H, W = msg.height, msg.width
        # distortion coefficients: plumb_bob has 5, pad/truncate anything else
        D = np.zeros(5)
        d = np.array(msg.d, dtype=float).reshape(-1)[:5]
        D[:d.size] = d
        # create new camera info
        self.camera_model = CameraModel(
            width=W,
            height=H,
            K=np.reshape(np.array(msg.k, dtype=float), (3, 3)),
            D=D,
            P=np.reshape(np.array(msg.p, dtype=float), (3, 4)),
        )
        self.rectifier = Rectifier(self.camera_model)
        self.rect_camera_K, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_model.K, self.camera_model.D, (W, H), 0.0
        )

    def cb_image(self, image_msg):
        self.latest_img = image_msg

    def doUpdate(self):
        """
        Correct the estimate with the AprilTags visible in the latest camera image.
        """
        if self.params.no_update:
            self.publish_pose()
            return

        if self.camera_model is None:
            return

        if self.latest_img is None:
            self.get_logger().info("waiting for first image", throttle_duration_sec=5.0)
            return

        # Decompress the image
        image_bgr = compressed_to_bgr(self.latest_img)

        # rect_image = self.rectifier.rectify(image_bgr, interpolation=cv2.INTER_CUBIC)

        # Convert to grayscale for AprilTag detection
        image_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

        # Camera parameters for pose estimation
        fx = self.camera_model.K[0, 0]
        fy = self.camera_model.K[1, 1]
        cx = self.camera_model.K[0, 2]
        cy = self.camera_model.K[1, 2]
        camera_params = [fx, fy, cx, cy]

        # AprilTag size in meters (Duckietown standard)
        tag_size = float(self.params.tag_size)

        # Detect AprilTags with pose estimation
        try:
            detections = self.apriltag_detector.detect(
                image_gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=tag_size
            )
        except Exception as e:
            self.get_logger().warn(f"AprilTag detection failed: {e}", throttle_duration_sec=5.0)
            return

        # Process each detection
        for detection in detections:
            tag_id = detection.tag_id

            # Check if this tag is in our map
            if tag_id not in self.map:
                continue

            # Get the tag position from the map
            tag_position = self.map[tag_id]
            tag_x, tag_y = tag_position[0], tag_position[1]

            # Get pose from detection (translation vector in camera frame)
            # pose_t is a 3x1 matrix: [x, y, z] where z is forward, x is right, y is down
            t = detection.pose_t

            # Extract range and bearing from the pose
            # Range is the distance to the tag
            range_estimate = np.linalg.norm(t)

            # Bearing is the angle in the horizontal plane (around y-axis)
            # arctan2(x, z) gives the angle from camera's forward direction
            bearing = -np.arctan2(t[0, 0], t[2, 0])
            bearing = wrap_angle(bearing)

            if self.params.test_sim:
                # let's calculate the exact range and bearing using the GT pose and
                # tag location (Duckiematrix only)
                if self.gt_pose is None:
                    continue
                dx = tag_x - self.gt_pose[0]
                dy = tag_y - self.gt_pose[1]
                range_estimate = np.linalg.norm([dx, dy])
                bearing = wrap_angle(np.arctan2(dy, dx) - self.gt_pose[2])

            # Update the EKF with this measurement
            self.ekf.update(np.array([range_estimate, bearing]), np.array([tag_x, tag_y]))
        ids = [det.tag_id for det in detections]
        self.last_detection_ids = ids
        self.publish_landmarks(ids)
        self.publish_detections(image_gray, detections, self.latest_img.header)
        self.publish_pose(self.latest_img.header)

    # --- publishers ---------------------------------------------------------------------------

    def publish_pose(self, header=None):
        pose_cov = PoseWithCovariance()
        pose_cov.pose.position.x = float(self.ekf.q[0])
        pose_cov.pose.position.y = float(self.ekf.q[1])
        pose_cov.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(self.ekf.q[2]))
        pose_cov.pose.orientation.x = qx
        pose_cov.pose.orientation.y = qy
        pose_cov.pose.orientation.z = qz
        pose_cov.pose.orientation.w = qw

        P = np.asarray(self.ekf.P, dtype=float)
        pose_cov.covariance = [
            P[0, 0], P[0, 1], 0.0, 0.0, 0.0, P[0, 2],
            P[1, 0], P[1, 1], 0.0, 0.0, 0.0, P[1, 2],
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            P[2, 0], P[2, 1], 0.0, 0.0, 0.0, P[2, 2],
        ]
        odom_msg = Odometry()
        if header is None:
            odom_msg.header.stamp = self.get_clock().now().to_msg()
        else:
            odom_msg.header.stamp = header.stamp
        odom_msg.header.frame_id = "map"
        odom_msg.pose = pose_cov

        self.pub_pose_covariance.publish(odom_msg)

    def resetParameters(self):
        self.get_logger().info("Encoder data resetting")
        self.delta_phi_left = 0.0
        self.left_tick_prev = None

        self.delta_phi_right = 0.0
        self.right_tick_prev = None

    def publish_landmarks(self, detection_ids):
        # publishes the whole map as markers and colors the ones that were detected
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for landmark_id, position in self.map.items():
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "landmarks"
            m.id = int(landmark_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(position[0])
            m.pose.position.y = float(position[1])
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            if landmark_id in detection_ids:
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 1.0
                m.color.a = 1.0
            else:
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.color.a = 1.0

            marker_array.markers.append(m)
        self.pub_landmark_markers.publish(marker_array)

    def publish_detections(self, img, detections, header):
        # get a color buffer from the BW image
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # draw each tag
        for detection in detections:
            for idx in range(len(detection.corners)):
                cv2.line(
                    img,
                    tuple(detection.corners[idx - 1, :].astype(int)),
                    tuple(detection.corners[idx, :].astype(int)),
                    (0, 255, 0),
                )
            # draw the tag ID
            cv2.putText(
                img,
                str(detection.tag_id),
                org=(detection.corners[0, 0].astype(int) + 10, detection.corners[0, 1].astype(int) + 10),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255),
            )
        # pack image into a message
        self.pub_detections.publish(bgr_to_compressed(img, header=header))


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

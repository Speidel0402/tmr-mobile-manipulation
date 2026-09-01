from __future__ import annotations

import json
import threading
from pathlib import Path

import numpy as np

import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from realsense2_camera_msgs.msg import Extrinsics

from .camera_geometry import depth_to_meters
from .config import load_config
from .math3d import pose_matrix
from .model import GroundedSAM2
from .pipeline import PerceptionPipeline
from .types import CameraIntrinsics, GraspResult
from .visualization import draw_overlay
from .validation import is_stale
from .selection import select_best_result


def _intrinsics(msg: CameraInfo) -> CameraIntrinsics:
    return CameraIntrinsics(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5], list(msg.d), msg.header.frame_id)


def _transform_matrix(msg) -> np.ndarray:
    t, q = msg.transform.translation, msg.transform.rotation
    return pose_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class RimGraspNode(Node):
    def __init__(self):
        super().__init__("rim_grasp_perception")
        self.declare_parameter("config", "")
        config_path = self.get_parameter("config").value
        if not config_path:
            raise RuntimeError("parameter 'config' must point to a camera YAML file")
        self.cfg = load_config(config_path)
        if self.cfg.require_common_frame_tf and not self.cfg.common_frame:
            raise RuntimeError("require_common_frame_tf=true but common_frame is empty")
        self.bridge = CvBridge()
        self.pipeline = PerceptionPipeline(self.cfg, GroundedSAM2(self.cfg.model))
        self.tf_buffer = Buffer(cache_time=RclDuration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.inference_lock = threading.Lock()
        self.last_input_s = 0.0
        self.last_valid = False
        self.dynamic_depth_to_color = None
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=3,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.rgb_sub = Subscriber(self, Image, self.cfg.topics.rgb, qos_profile=reliable)
        self.depth_sub = Subscriber(self, Image, self.cfg.topics.depth, qos_profile=reliable)
        self.rgb_info_sub = Subscriber(self, CameraInfo, self.cfg.topics.rgb_info, qos_profile=reliable)
        self.depth_info_sub = Subscriber(self, CameraInfo, self.cfg.topics.depth_info, qos_profile=reliable)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.rgb_info_sub, self.depth_info_sub], queue_size=8,
            slop=self.cfg.geometry.sync_slop_s, allow_headerless=False)
        self.sync.registerCallback(self._callback)
        self.json_pub = self.create_publisher(String, self.cfg.topics.result_json, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.cfg.topics.contact_pose, 10)
        self.overlay_pub = self.create_publisher(Image, self.cfg.topics.overlay, 2)
        self.marker_pub = self.create_publisher(MarkerArray, self.cfg.topics.markers, 2)
        extrinsics_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                    reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.extrinsics_sub = self.create_subscription(
            Extrinsics, self.cfg.topics.depth_to_color_extrinsics,
            self._extrinsics_callback, extrinsics_qos)
        self.timer = self.create_timer(max(0.1, self.cfg.geometry.target_timeout_s / 3), self._expire)
        self.get_logger().info(f"camera={self.cfg.camera_id}; local model loading complete")

    def _callback(self, rgb_msg, depth_msg, rgb_info, depth_info):
        if not self.inference_lock.acquire(blocking=False):
            return
        try:
            stamp_ns = rgb_msg.header.stamp.sec * 1_000_000_000 + rgb_msg.header.stamp.nanosec
            stamp_s = stamp_ns / 1e9
            self.last_input_s = self.get_clock().now().nanoseconds / 1e9
            if self.cfg.camera_optical_frame and rgb_msg.header.frame_id != self.cfg.camera_optical_frame:
                self._publish_json([GraspResult(stamp_s, "", self.cfg.camera_id, "", "", False,
                                                 "unexpected_rgb_optical_frame",
                                                 diagnostics={"received": rgb_msg.header.frame_id,
                                                              "configured": self.cfg.camera_optical_frame})])
                return
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth = depth_to_meters(raw_depth, depth_msg.encoding, self.cfg.geometry.depth_scale_m)
            d2c = self._depth_to_color()
            output_tf, output_frame, tf_reason = self._lookup_output_tf(rgb_msg.header.frame_id, stamp_ns)
            if tf_reason:
                results = [GraspResult(stamp_s, self.cfg.common_frame, self.cfg.camera_id, "", "", False, tf_reason)]
                debug = {}
            else:
                results, debug = self.pipeline.process(
                    rgb, depth, _intrinsics(rgb_info), _intrinsics(depth_info), stamp_s,
                    d2c, output_tf, output_frame)
            self._publish(results, rgb_msg, rgb, debug)
        except Exception as exc:
            self.get_logger().error(f"frame rejected: {type(exc).__name__}: {exc}")
            now = self.get_clock().now().nanoseconds / 1e9
            self._publish_json([GraspResult(now, "", self.cfg.camera_id, "", "", False, "processing_exception",
                                             diagnostics={"exception_type": type(exc).__name__})])
        finally:
            self.inference_lock.release()

    def _depth_to_color(self):
        if self.cfg.depth_is_aligned_to_rgb:
            return np.eye(4)
        value = self.cfg.depth_to_color
        if value:
            out = np.eye(4)
            out[:3, :3] = np.asarray(value["rotation_row_major"], float).reshape(3, 3)
            out[:3, 3] = np.asarray(value["translation_m"], float)
            return out
        return self.dynamic_depth_to_color

    def _extrinsics_callback(self, msg: Extrinsics):
        out = np.eye(4)
        out[:3, :3] = np.asarray(msg.rotation, dtype=float).reshape(3, 3)
        out[:3, 3] = np.asarray(msg.translation, dtype=float)
        self.dynamic_depth_to_color = out

    def _lookup_output_tf(self, camera_frame, stamp_ns):
        target = self.cfg.common_frame
        if not target or target == camera_frame:
            return None, camera_frame, ""
        try:
            tf = self.tf_buffer.lookup_transform(target, camera_frame, rclpy.time.Time(nanoseconds=stamp_ns),
                                                  timeout=RclDuration(seconds=self.cfg.geometry.tf_timeout_s))
        except TransformException:
            return None, target, "tf_unavailable_at_image_stamp"
        tf_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
        if tf_ns and abs(stamp_ns - tf_ns) / 1e9 > self.cfg.geometry.tf_max_age_s:
            return None, target, "tf_too_old_for_image_stamp"
        return _transform_matrix(tf), target, ""

    def _publish(self, results, rgb_msg, rgb, debug):
        selected = select_best_result(results)
        self._publish_json(results)
        self.last_valid = selected is not None
        if selected is not None:
            p = PoseStamped()
            p.header = rgb_msg.header
            p.header.frame_id = selected.frame_id
            v = selected.contact_pose
            p.pose.position.x, p.pose.position.y, p.pose.position.z = v.position
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = v.orientation_xyzw
            self.pose_pub.publish(p)
        overlay = draw_overlay(rgb, results, debug)
        msg = self.bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
        msg.header = rgb_msg.header
        self.overlay_pub.publish(msg)
        self.marker_pub.publish(self._markers(results, rgb_msg.header.stamp))

    def _publish_json(self, results):
        msg = String()
        msg.data = json.dumps([r.to_dict() for r in results], ensure_ascii=False, separators=(",", ":"))
        self.json_pub.publish(msg)

    def _markers(self, results, stamp):
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        mid = 0
        for r in results:
            if not r.valid or not r.diagnostics.get("selected", False):
                continue
            for name, direction, color in (("approach", r.approach_direction, (0., 0.5, 1., 1.)),
                                            ("closing", r.closing_direction, (1., 0.4, 0., 1.))):
                m = Marker(); m.header.frame_id = r.frame_id; m.header.stamp = stamp
                m.ns = f"{self.cfg.camera_id}_{name}"; m.id = mid; mid += 1
                m.type = Marker.ARROW; m.action = Marker.ADD; m.scale.x = .006; m.scale.y = .012; m.scale.z = .012
                m.color.r, m.color.g, m.color.b, m.color.a = color
                start = r.contact_pose.position; end = np.asarray(start) + .06*np.asarray(direction)
                m.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                            Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
                m.lifetime = Duration(sec=0, nanosec=int(self.cfg.geometry.target_timeout_s*1e9))
                arr.markers.append(m)
        return arr

    def _expire(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if is_stale(self.last_input_s, now, self.cfg.geometry.target_timeout_s) and self.last_valid:
            self.last_valid = False
            self._publish_json([GraspResult(now, "", self.cfg.camera_id, "", "", False, "target_stale_timeout")])
            arr = MarkerArray(); m = Marker(); m.action = Marker.DELETEALL; arr.markers.append(m); self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RimGraspNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

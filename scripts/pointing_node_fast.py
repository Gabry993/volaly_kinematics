#!/usr/bin/env python3

from typing import Any, Optional
import sys
import yaml
import math

# import tf2_ros

import PyKDL
import rclpy
import rclpy.node

from geometry_msgs.msg import QuaternionStamped, Pose, PoseStamped, Transform
# from geometry_msgs.msg import TransformStamped


class PointingModel:

    def __init__(self, footprint_to_neck: float, neck_to_eyes: float,
                 shoulder_to_wrist: float, wrist_to_finger: float, **kwargs: Any) -> None:
        self.eye_position = PyKDL.Vector(0, 0, footprint_to_neck + neck_to_eyes)
        self.shoulder_position = PyKDL.Vector(0, 0, footprint_to_neck)
        self.shoulder_to_finger = wrist_to_finger + shoulder_to_wrist

    def ray(self, wrist_orientation: PyKDL.Rotation) -> PyKDL.Frame:
        # The wrist x-axis is oriented along the arm
        finger_position = (self.shoulder_position +
                           wrist_orientation * PyKDL.Vector(self.shoulder_to_finger, 0, 0))
        # The ray is aligned to its x-axis
        ray_axis = finger_position - self.eye_position
        ray_axis = ray_axis / ray_axis.Norm()
        # rot is the rotation that align the x-axis with the ray
        rot_angle = math.acos(PyKDL.dot(PyKDL.Vector(1, 0, 0), ray_axis))
        rot_axis = PyKDL.Vector(1, 0, 0) * ray_axis
        return PyKDL.Frame(R=PyKDL.Rotation.Rot(rot_axis, rot_angle), V=self.eye_position)


def pose_from_frame(frame: PyKDL.Frame) -> Pose:
    msg = Pose()
    o = msg.orientation
    o.x, o.y, o.z, o.w = frame.M.GetQuaternion()
    p = msg.position
    p.x, p.y, p.z = frame.p
    return msg


def transform_from_frame(frame: PyKDL.Frame) -> Transform:
    msg = Transform()
    o = msg.rotation
    o.x, o.y, o.z, o.w = frame.M.GetQuaternion()
    p = msg.translation
    p.x, p.y, p.z = frame.p
    return msg


class PointingNode(rclpy.node.Node):  # type: ignore

    def __init__(self, node_name: str = 'pointing_node') -> None:
        super().__init__(node_name)

        biometrics_path = self.declare_parameter("biometrics", "").value
        with open(biometrics_path, 'r') as f:
            biometrics = yaml.safe_load(f)

        self.pointing_model = PointingModel(**biometrics)
        maximal_publish_rate = self.declare_parameter("publish_rate", 50.0).value
        self.human_frame = self.declare_parameter(
            "human_frame", "human_footprint"
        ).value

        self.last_timestamp: Optional[rclpy.time.Time] = None
        pointing_ray_topic = self.declare_parameter(
            "pointing_ray_topic", "pointing_ray"
        ).value
        self.pub_pointing_ray = self.create_publisher(
            PoseStamped, pointing_ray_topic, 100
        )

        # self.tf_buff = tf2_ros.Buffer()
        # self.tf_br = tf2_ros.TransformBroadcaster(node=self)
        rotation_topic = self.declare_parameter(
            "rotation_topic", "/metawear_ros/rotation"
        ).value
        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )

        self.sub_rotation = self.create_subscription(
            QuaternionStamped,
            rotation_topic,
            self.rotation_to_ray_cb,
            qos,
        )

        self.min_interval = rclpy.duration.Duration(nanoseconds=int(1e9 / maximal_publish_rate))

    def rotation_to_ray_cb(self, data: QuaternionStamped) -> None:
        stamp = rclpy.time.Time.from_msg(data.header.stamp)
        if self.last_timestamp and (stamp - self.last_timestamp) < self.min_interval:
            self.last_timestamp = stamp
            return
        self.last_timestamp = stamp

        rot = PyKDL.Rotation.Quaternion(
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w,
        )
        ray_frame = self.pointing_model.ray(rot)
        # TODO(Jerome): do we need the transform?
        # transform_msg = TransformStamped()
        # transform_msg.child_frame_id = "pointer"
        # transform_msg.header = data.header
        # transform_msg.transform = transform_from_frame(ray_frame)
        # self.tf_br.sendTransform(transform_msg)

        ray_pose = PoseStamped()
        ray_pose.header.stamp = data.header.stamp
        ray_pose.header.frame_id = self.human_frame
        ray_pose.pose = pose_from_frame(ray_frame)
        self.pub_pointing_ray.publish(ray_pose)


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = PointingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

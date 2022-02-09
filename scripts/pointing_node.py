#!/usr/bin/env python3

import math
import copy
import sys

import numpy as np
import tf2_ros

# import tf_conversions as tfc
import PyKDL as kdl
import rclpy
from rclpy.exceptions import ParameterException, ROSInterruptException

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, PoseStamped, QuaternionStamped
from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker

from volaly_kinematics.pointing_model import HeadFingerModel
from volaly_kinematics.workspace_shape import Plane, Cylinder, Sphere
from volaly_msgs.srv import SetWorkspaceShape


class PointingNode(rclpy.node.Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # self.publish_rate = rospy.get_param('~publish_rate', 50.0)
        self.publish_rate = self.declare_parameter("publish_rate", 50.0).value

        # self.ray_origin_frame = rospy.get_param('~ray_origin_frame', 'eyes')
        self.ray_origin_frame = self.declare_parameter(
            "ray_origin_frame", "eyes"
        ).value
        # self.ray_pass_frame = rospy.get_param('~ray_pass_frame', 'finger')
        self.ray_pass_frame = self.declare_parameter(
            "ray_pass_frame", "finger"
        ).value

        # self.human_frame = rospy.get_param('~human_frame', 'human_footprint')
        self.human_frame = self.declare_parameter(
            "human_frame", "human_footprint"
        ).value

        # self.user_prefix = self.declare_parameter(
        #     "user_prefix", ""
        # ).value
        # self.user_prefix += "/" if self.user_prefix != "" else ""
        # self.human_frame = self.user_prefix + self.human_frame

        self.robot_frame = self.declare_parameter(
            "robot_frame", "robot_base_link"
        ).value

        # arm_pointer_topic = rospy.get_param('~arm_pointer_topic', 'arm_pointer')
        arm_pointer_topic = self.declare_parameter(
            "arm_pointer_topic", "arm_pointer"
        ).value
        # self.pub_arm_pointer = rospy.Publisher(arm_pointer_topic, PoseStamped, queue_size = 100)
        self.pub_arm_pointer = self.create_publisher(
            PoseStamped, arm_pointer_topic, 100
        )

        # human_pose_topic = rospy.get_param('~human_pose_topic', 'human_pose')
        human_pose_topic = self.declare_parameter(
            "human_pose_topic", "human_pose"
        ).value
        self.pub_human_pose = self.create_publisher(
            PoseStamped, human_pose_topic, 100
        )

        pointing_ray_topic = self.declare_parameter(
            "pointing_ray_topic", "pointing_ray"
        ).value
        self.pub_pointing_ray = self.create_publisher(
            PoseStamped, pointing_ray_topic, 100
        )

        self.pointer_switch_margin = self.declare_parameter(
            "pointer_switch_margin", 0.10
        ).value  # 10 cm
        self.switch_at_pointer = self.declare_parameter(
            "switch_at_pointer", False
        ).value

        min_elev_angle_deg = self.declare_parameter(
            "min_elev_angle_deg", 5.0
        ).value  # 5 deg
        self.min_elevation_angle = math.radians(min_elev_angle_deg)

        markers_topic = self.declare_parameter(
            "markers_topic", "markers"
        ).value
        self.pub_markers = self.create_publisher(Marker, markers_topic, 10)

        self.tf_buff = tf2_ros.Buffer()
        self.tf_ls = tf2_ros.TransformListener(self.tf_buff, node=self)
        self.tf_br = tf2_ros.TransformBroadcaster(node=self)

        self.pointing_model = HeadFingerModel(
            self.tf_buff,
            self.human_frame,
            self.ray_origin_frame,
            self.ray_pass_frame,
            "pointer",
        )

        self.xy_plane = Plane(
            self.human_frame,
            normal=kdl.Vector(0.0, 0.0, 1.0),
            point=kdl.Vector(0.0, 0.0, 0.6),
        )
        self.vis_plane = Plane(self.human_frame)
        self.cylinder = Cylinder(
            self.human_frame, axis=kdl.Vector(0.0, 0.0, 1.0)
        )
        self.sphere = Sphere(self.ray_origin_frame)

        self.ws_shape = self.xy_plane
        # self.ws_shape = self.cylinder
        # self.ws_shape = self.sphere
        self.last_ws_shape = None

        default_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_ALL
        )
        # self.srv_set_workspace_shape = rospy.Service('set_workspace_shape', SetWorkspaceShape, self.set_workspace_shape)
        self.srv_set_workspace_shape = self.create_service(
            SetWorkspaceShape, "set_workspace_shape", self.set_workspace_shape
        )
        self.sub_xy_plane_height = self.create_subscription(
            Float32, "xy_plane_height", self.set_xy_plane_height, default_qos
        )
        qos = rclpy.qos.QoSProfile(
            depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_workspace_shape = self.create_publisher(
            String, "workspace_shape", qos
        )

        #### Experimental ####
        joint_states_topic = self.declare_parameter(
            "joint_states_topic", "imu/joint_states"
        ).value
        self.joint_prefix = self.declare_parameter(
            "joint_prefix", "shoulder_to_wrist"
        ).value
        ######################
        self.pub_joint_state = self.create_publisher(
            JointState, joint_states_topic, 10
        )

        rotation_topic = self.declare_parameter(
            "rotation_topic", "/metawear_ros/rotation"
        ).value

        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_rotation = self.create_subscription(
            QuaternionStamped,
            rotation_topic,
            self.rotation_to_ray_cb,
            qos,
        )

        self.create_subscription(
            PoseStamped,
            "/optitrack/hand",
            self.optitrack_imu_cb,
            100,
        )

        qos = rclpy.qos.QoSProfile(
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
        )
        self.optitrack_imu_pub = self.create_publisher(
            QuaternionStamped, "/optitrack/imu", qos
        )

        timer_period = 1 / self.publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.run)

    def optitrack_imu_cb(self, data):
        msg = QuaternionStamped()
        msg.header = data.header
        msg.header.frame_id = self.human_frame
        msg.quaternion = data.pose.orientation
        self.optitrack_imu_pub.publish(msg)

    def rotation_to_ray_cb(self, data):
        rot = kdl.Rotation.Quaternion(
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w,
        )
        (yaw, pitch, roll) = rot.GetEulerZYX()
        j_state = JointState()
        j_state.header.stamp = data.header.stamp  # now
        j_state.name = [
            self.joint_prefix + "_roll",
            self.joint_prefix + "_pitch",
            self.joint_prefix + "_yaw",
        ]
        j_state.position = [roll, pitch, yaw]
        self.pub_joint_state.publish(j_state)

    def set_xy_plane_height(self, msg):
        self.xy_plane.point = kdl.Vector(0.0, 0.0, msg.data)

    def get_pointer(
        self, ws_shape, ray_kdl_frame, pass_point=None, cache=False
    ):
        robot_kdl_frame = self.frame_from_tf(
            self.human_frame, self.robot_frame
        )

        if not pass_point:
            if not ws_shape.point:
                if robot_kdl_frame:
                    # Initialize with robot's current position
                    ws_shape.point = copy.deepcopy(robot_kdl_frame.p)
                else:
                    return None

            pass_point = ws_shape.point

        ray_origin_kdl_frame = self.frame_from_tf(
            self.human_frame, self.ray_origin_frame
        )
        shape_origin_kdl_frame = self.frame_from_tf(
            self.human_frame, ws_shape.frame_id
        )

        pointer_pose = None

        if pass_point and ray_origin_kdl_frame and shape_origin_kdl_frame:
            actual_pass_point = pass_point - copy.deepcopy(
                shape_origin_kdl_frame.p
            )

            if cache:  # or not ws_shape.point:
                ws_shape.point = pass_point

            if isinstance(ws_shape, Plane):
                normal = None

                if ws_shape is self.vis_plane:
                    tmp_point = copy.deepcopy(ray_origin_kdl_frame.p)
                    robot_dir = actual_pass_point - tmp_point
                    normal = kdl.Vector(robot_dir.x(), robot_dir.y(), 0.0)

                    if cache:
                        ws_shape.normal = normal

                pointer_pose = ws_shape.ray_to_location(
                    ray_kdl_frame, normal=normal, point=actual_pass_point
                )
            else:
                pointer_pose = ws_shape.ray_to_location(
                    ray_kdl_frame, point=actual_pass_point
                )

        return pointer_pose

    def transform_to_kdl(self, t):
        return kdl.Frame(
            kdl.Rotation.Quaternion(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ),
            kdl.Vector(
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ),
        )

    def tf_from_pose(self, p):
        """
        :param p: input pose
        :type p: :class:`geometry_msgs.msg.Pose`
        :return: New :class:`PyKDL.Frame` object
        Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
        """
        return kdl.Frame(
            kdl.Rotation.Quaternion(
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            ),
            kdl.Vector(p.position.x, p.position.y, p.position.z),
        )

    def tf_to_pose(self, f):
        """
        :param f: input pose
        :type f: :class:`PyKDL.Frame`
        Return a ROS Pose message for the Frame f.
        """
        p = Pose()
        (
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w,
        ) = f.M.GetQuaternion()
        p.position.x = f.p[0]
        p.position.y = f.p[1]
        p.position.z = f.p[2]
        return p

    def set_workspace_shape(self, req, response):
        ray_tf = self.pointing_model.pointing_ray()

        req_ws_shape = self.ws_shape

        if req.robot_frame:
            self.robot_frame = req.robot_frame

        if ray_tf:
            ray_kdl_frame = self.transform_to_kdl(ray_tf)
            robot_kdl_frame = self.frame_from_tf(
                self.human_frame, self.robot_frame
            )  # self.ws_shape.frame_id, self.robot_frame)
            ray_origin_kdl_frame = self.frame_from_tf(
                self.human_frame, self.ray_origin_frame
            )

            if robot_kdl_frame and ray_origin_kdl_frame:
                if req.workspace_shape == req.WORKSPACE_XY_PLANE:
                    req_ws_shape = self.xy_plane
                elif req.workspace_shape == req.WORKSPACE_VISUAL_PLANE:
                    req_ws_shape = self.vis_plane
                elif req.workspace_shape == req.WORKSPACE_CYLINDER:
                    req_ws_shape = self.cylinder
                elif req.workspace_shape == req.WORKSPACE_SPHERE:
                    req_ws_shape = self.sphere
                else:
                    raise ParameterException(
                        "Unsupported shape type", req.workspace_shape
                    )
                    # return None

                cur_pointer_pose = self.get_pointer(
                    self.ws_shape, ray_kdl_frame, self.ws_shape.point
                )

                if self.switch_at_pointer:
                    switch_point = copy.deepcopy(
                        self.tf_from_pose(cur_pointer_pose).p
                    )
                else:
                    switch_point = copy.deepcopy(robot_kdl_frame.p)

                new_pointer_pose = self.get_pointer(
                    req_ws_shape, ray_kdl_frame, switch_point
                )

                eyes_robot_vector = kdl.Vector(
                    robot_kdl_frame.p - ray_origin_kdl_frame.p
                )

                robot_elev_frame = self.pointing_model._frame_from_direction(
                    ray_origin_kdl_frame.p, eyes_robot_vector
                )
                _, pitch, _ = robot_elev_frame.M.GetRPY()

                if math.fabs(pitch) < self.min_elevation_angle:
                    raise ROSInterruptException(
                        "Drone elevation angle is less than {} deg: {}. Ignoring".format(
                            math.degrees(self.min_elevation_angle),
                            math.degrees(pitch),
                        )
                    )
                else:
                    # Since the safety check succeeded we can update the pass_point of the shape
                    new_pointer_pose = self.get_pointer(
                        req_ws_shape, ray_kdl_frame, switch_point, cache=True
                    )

            else:
                raise ROSInterruptException(
                    "Unable to obtain robot's frame. Is robot localized?"
                )
                # return None

        else:
            raise ROSInterruptException(
                "Internal error: failed to get ray frame"
            )
            # return None

        self.ws_shape = req_ws_shape

        return response

    def frame_from_tf(self, fixed_frame, target_frame):
        if fixed_frame == target_frame:
            return kdl.Frame.Identity()

        try:
            tf_frame = self.tf_buff.lookup_transform(
                fixed_frame, target_frame, rclpy.time.Time()
            )
            dt = self.get_clock().now() - rclpy.time.Time.from_msg(
                tf_frame.header.stamp
            )
            if dt > rclpy.duration.Duration(seconds=5.0):
                self.get_logger().warn(
                    "Transformation [{}] -> [{}] is too old. Last seen {:.3f}s ago. Ignoring".format(
                        fixed_frame, target_frame, dt.nanoseconds / 1e-9
                    )
                )
                return None
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            # self.get_logger().error(str(e), throttle_duration_sec=5)
            return None

        kdl_frame = self.transform_to_kdl(tf_frame)

        return kdl_frame

    def frame_from_direction(self, orig_v, dir_v):
        u = kdl.Vector(dir_v.x(), dir_v.y(), dir_v.z())
        u.Normalize()

        yaw = kdl.Rotation.RPY(0.0, 0.0, math.atan2(u.y(), u.x()))
        # Convention: pointing ray is along Z-axis
        new_x = yaw * kdl.Vector(1.0, 0.0, 0.0)
        pitch = kdl.Rotation.RPY(
            0.0, math.atan2(-u.z(), kdl.dot(u, new_x)), 0.0
        )

        frame = kdl.Frame(yaw * pitch, orig_v)

        return frame

    def plane_pose(self, origin, normal):
        frame = self.frame_from_direction(origin, normal)
        # Make Z-axis to lie along X-axis
        new_frame = frame.M * kdl.Rotation.RPY(0.0, math.pi / 2.0, 0.0)
        qx, qy, qz, qw = new_frame.GetQuaternion()

        pose = Pose()
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        pose.position.x = origin.x()
        pose.position.y = origin.y()
        pose.position.z = origin.z()

        return pose

    def create_rviz_marker(self, ws_shape, vector):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = ws_shape.frame_id
        m.lifetime = rclpy.duration.Duration().to_msg()
        m.ns = "workspace_shape"
        m.id = 0
        m.pose.orientation.w = 1.0

        m.color.a = 0.3
        m.color.r = 0.0
        m.color.g = 0.3
        m.color.b = 0.0

        m.action = Marker.ADD

        if isinstance(ws_shape, Plane):
            m.type = Marker.CUBE
            m.scale.x = 7.0
            m.scale.y = 7.0
            m.scale.z = 0.001

            # # m.pose.position.z = vector.z()
            m.pose = self.plane_pose(vector, ws_shape.normal)
            # m.pose.position.z = -m.scale.x / 2.0

        elif isinstance(ws_shape, Cylinder):
            m.type = Marker.CYLINDER

            diam = kdl.Vector(vector.x(), vector.y(), 0.0).Norm() * 2.0

            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = 7.0
            m.pose.position.z = m.scale.z / 2.0

        elif isinstance(ws_shape, Sphere):
            m.type = Marker.SPHERE

            diam = vector.Norm() * 2.0

            m.scale.x = diam
            m.scale.y = diam
            m.scale.z = diam

        return m

    def create_pointing_ray_marker(self, frame_id, lenght=10.0):
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        # m.frame_locked = True
        m.lifetime = rclpy.duration.Duration().to_msg()
        m.ns = "pointing_ray"
        m.id = 0
        m.action = Marker.ADD
        m.type = Marker.CYLINDER

        frame1 = kdl.Frame(
            kdl.Rotation.RPY(0.0, math.pi / 2.0, 0.0),
            kdl.Vector(0.0, 0.0, 0.0),
        )
        frame2 = kdl.Frame(
            kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, lenght / 2.0)
        )
        # First, move, then rotate
        frame = frame1 * frame2
        qx, qy, qz, qw = frame.M.GetQuaternion()

        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        m.pose.position.x = frame.p.x()
        m.pose.position.y = frame.p.y()
        m.pose.position.z = frame.p.z()

        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0

        diam = 0.01

        m.scale.x = diam
        m.scale.y = diam
        m.scale.z = lenght

        return m

    def get_shape_name(self, ws_shape):
        if ws_shape is self.xy_plane:
            return "WORKSPACE_XY_PLANE"
        elif ws_shape is self.vis_plane:
            return "WORKSPACE_VISUAL_PLANE"
        elif ws_shape is self.cylinder:
            return "WORKSPACE_CYLINDER"
        elif ws_shape is self.sphere:
            return "WORKSPACE_SPHERE"

    def run(self):
        if self.ws_shape:
            if self.ws_shape != self.last_ws_shape:
                self.pub_workspace_shape.publish(
                    String(data=self.get_shape_name(self.ws_shape))
                )
                self.last_ws_shape = self.ws_shape

        ray_tf = self.pointing_model.pointing_ray()

        if ray_tf:
            human_pose_msg = PoseStamped()
            human_pose_msg.header = ray_tf.header
            human_pose_msg.pose.position.x = 0.0
            human_pose_msg.pose.position.y = 0.0
            human_pose_msg.pose.position.z = 0.0
            self.pub_human_pose.publish(human_pose_msg)
            ray_kdl_frame = self.transform_to_kdl(ray_tf)
            self.tf_br.sendTransform(ray_tf)

            ray_pose = PoseStamped()
            ray_pose.header = ray_tf.header
            ray_pose.pose = self.tf_to_pose(ray_kdl_frame)
            self.pub_pointing_ray.publish(ray_pose)

            ray_origin_kdl_frame = self.frame_from_tf(
                self.human_frame, self.ray_origin_frame
            )
            shape_origin_kdl_frame = self.frame_from_tf(
                self.human_frame, self.ws_shape.frame_id
            )

            if ray_origin_kdl_frame and shape_origin_kdl_frame:
                pointer_pose = self.get_pointer(self.ws_shape, ray_kdl_frame)
                if pointer_pose:
                    m_msg = self.create_rviz_marker(
                        self.ws_shape,
                        self.ws_shape.point
                        - copy.deepcopy(shape_origin_kdl_frame.p),
                    )

                    if m_msg:
                        self.pub_markers.publish(m_msg)

                    pose_msg = PoseStamped()
                    pose_msg.header = ray_tf.header
                    pose_msg.pose = pointer_pose

                    self.pub_arm_pointer.publish(pose_msg)

                    eyes_pointer = (
                        self.tf_from_pose(pointer_pose).p
                        - ray_origin_kdl_frame.p
                    )
                    pr_marker_msg = self.create_pointing_ray_marker(
                        self.pointing_model.frame_id, eyes_pointer.Norm()
                    )

                    if pr_marker_msg:
                        self.pub_markers.publish(pr_marker_msg)


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = PointingNode(node_name="pointing_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

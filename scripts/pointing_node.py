#!/usr/bin/env python

import math
import copy

import rospy
import tf2_ros
import tf_conversions as tfc
import PyKDL as kdl

from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs import transform_to_kdl
from visualization_msgs.msg import Marker

from volaly_kinematics.pointing_model import HeadFingerModel
from volaly_kinematics.workspace_shape import Plane, Cylinder, Sphere
from volaly_msgs.srv import SetWorkspaceShape, SetWorkspaceShapeRequest, SetWorkspaceShapeResponse

class PointingNode:
    def __init__(self):
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)

        self.ray_origin_frame = rospy.get_param('~ray_origin_frame', 'eyes')
        self.ray_pass_frame = rospy.get_param('~ray_pass_frame', 'finger')

        self.human_frame = rospy.get_param('~human_frame', 'human_footprint')
        self.robot_frame = rospy.get_param('~robot_frame', 'robot_base_link')

        arm_pointer_topic = rospy.get_param('~arm_pointer_topic', 'arm_pointer')
        self.pub_arm_pointer = rospy.Publisher(arm_pointer_topic, PoseStamped, queue_size = 100)

        pointing_ray_topic = rospy.get_param('~pointing_ray_topic', 'pointing_ray')
        self.pub_pointing_ray = rospy.Publisher(pointing_ray_topic, PoseStamped, queue_size = 100)

        self.pointer_switch_margin = rospy.get_param('~pointer_switch_margin', 0.10) # 10 cm
        self.switch_at_pointer = rospy.get_param('~switch_at_pointer', False)

        markers_topic = rospy.get_param('~markers_topic', 'markers')
        self.pub_markers = rospy.Publisher(markers_topic, Marker, queue_size = 10)

        self.tf_buff = tf2_ros.Buffer()
        self.tf_ls = tf2_ros.TransformListener(self.tf_buff)
        self.tf_br = tf2_ros.TransformBroadcaster()

        self.pointing_model = HeadFingerModel(self.tf_buff, self.human_frame, self.ray_origin_frame, self.ray_pass_frame, 'pointer')

        self.xy_plane = Plane(self.human_frame, normal=kdl.Vector(0.0, 0.0, 1.0)) #, point=kdl.Vector(0.0, 0.0, 0.6))
        self.vis_plane = Plane(self.human_frame)
        self.cylinder = Cylinder(self.human_frame, axis=kdl.Vector(0.0, 0.0, 1.0))
        self.sphere = Sphere(self.ray_origin_frame)

        self.ws_shape = self.xy_plane
        # self.ws_shape = self.cylinder
        # self.ws_shape = self.sphere

        self.srv_set_workspace_shape = rospy.Service('set_workspace_shape', SetWorkspaceShape, self.set_workspace_shape)

    def get_pointer(self, ws_shape, ray_kdl_frame, pass_point=None, cache=False):
        robot_kdl_frame = self.frame_from_tf(self.human_frame, self.robot_frame)

        if not pass_point:
            if not ws_shape.point:
                if robot_kdl_frame:
                    # Initialize with robot's current position
                    ws_shape.point = copy.deepcopy(robot_kdl_frame.p)
                else:
                    return None

            pass_point = ws_shape.point


        ray_origin_kdl_frame = self.frame_from_tf(self.human_frame, self.ray_origin_frame)
        shape_origin_kdl_frame = self.frame_from_tf(self.human_frame, ws_shape.frame_id)

        pointer_pose = None

        if pass_point and ray_origin_kdl_frame and shape_origin_kdl_frame:
            actual_pass_point = pass_point - copy.deepcopy(shape_origin_kdl_frame.p)

            if cache: # or not ws_shape.point:
                ws_shape.point = pass_point

            if isinstance(ws_shape, Plane):
                normal = None

                if ws_shape is self.vis_plane:
                    tmp_point = copy.deepcopy(ray_origin_kdl_frame.p)
                    robot_dir = actual_pass_point - tmp_point
                    normal = kdl.Vector(robot_dir.x(), robot_dir.y(), 0.0)

                    if cache:
                        ws_shape.normal = normal

                pointer_pose = ws_shape.ray_to_location(ray_kdl_frame, normal=normal, point=actual_pass_point)
            else:
                pointer_pose = ws_shape.ray_to_location(ray_kdl_frame, point=actual_pass_point)

        return pointer_pose

    def set_workspace_shape(self, req):
        ray_tf = self.pointing_model.pointing_ray()

        req_ws_shape = self.ws_shape

        if req.robot_frame:
            self.robot_frame = req.robot_frame

        if ray_tf:
            ray_kdl_frame = transform_to_kdl(ray_tf)
            robot_kdl_frame = self.frame_from_tf(self.human_frame, self.robot_frame) #self.ws_shape.frame_id, self.robot_frame)
            ray_origin_kdl_frame = self.frame_from_tf(self.human_frame, self.ray_origin_frame)

            if robot_kdl_frame and ray_origin_kdl_frame:
                if req.workspace_shape == SetWorkspaceShapeRequest.WORKSPACE_XY_PLANE:
                    req_ws_shape = self.xy_plane
                elif req.workspace_shape == SetWorkspaceShapeRequest.WORKSPACE_VISUAL_PLANE:
                    req_ws_shape = self.vis_plane
                elif req.workspace_shape == SetWorkspaceShapeRequest.WORKSPACE_CYLINDER:
                    req_ws_shape = self.cylinder
                elif req.workspace_shape == SetWorkspaceShapeRequest.WORKSPACE_SPHERE:
                    req_ws_shape = self.sphere
                else:
                    raise rospy.ServiceException('Unsupported shape type')
                    # return None

                cur_pointer_pose = self.get_pointer(self.ws_shape, ray_kdl_frame, self.ws_shape.point)

                if self.switch_at_pointer:
                    switch_point = copy.deepcopy(tfc.fromMsg(cur_pointer_pose).p)
                else:
                    switch_point = copy.deepcopy(robot_kdl_frame.p)


                new_pointer_pose = self.get_pointer(req_ws_shape, ray_kdl_frame, switch_point)

                eyes_robot_vector = kdl.Vector(robot_kdl_frame.p - ray_origin_frame.p)

                robot_elev_frame = self.pointing_model._frame_from_direction(ray_origin_frame.p, eyes_robot_vector)
                _, pitch, _ = robot_elev_frame.M.GetRPY()

                if math.fabs(math.degrees(pitch)) < 5.0:
                    raise rospy.ServiceException('Drone elevation angle is less than {} deg: {}. Ignoring'.format(math.degrees(pitch)))

            else:
                raise rospy.ServiceException('Unable to obtain robot\'s frame. Is robot localized?')
                # return None

        else:
            raise rospy.ServiceException('Internal error: failed to get ray frame')
            # return None

        self.ws_shape = req_ws_shape

        return SetWorkspaceShapeResponse()

    def frame_from_tf(self, fixed_frame, target_frame):
        if fixed_frame == target_frame:
            return kdl.Frame.Identity()

        try:
            tf_frame = self.tf_buff.lookup_transform(fixed_frame, target_frame, rospy.Time())
            dt = (rospy.Time.now() - tf_frame.header.stamp)
            if  dt > rospy.Duration(5.0):
                rospy.logwarn_throttle(10.0, 'Transformation [{}] -> [{}] is too old. Last seen {:.3f}s ago. Ignoring'
                    .format(fixed_frame, target_frame, dt.to_sec()))
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
            rospy.logerr_throttle(10.0, e.message)
            return None

        kdl_frame = transform_to_kdl(tf_frame)

        return kdl_frame

    def frame_from_direction(self, orig_v, dir_v):
        u = kdl.Vector(dir_v.x(), dir_v.y(), dir_v.z())
        u.Normalize()

        yaw = kdl.Rotation.RPY(0.0, 0.0, math.atan2(u.y(), u.x()))
        # Convention: pointing ray is along Z-axis
        new_x = yaw * kdl.Vector(1.0, 0.0, 0.0)
        pitch = kdl.Rotation.RPY(0.0, math.atan2(-u.z(), kdl.dot(u, new_x)), 0.0)

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
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = ws_shape.frame_id
        m.lifetime = rospy.Duration()
        m.ns = 'workspace_shape'
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
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = frame_id
        # m.frame_locked = True
        m.lifetime = rospy.Duration()
        m.ns = 'pointing_ray'
        m.id = 0
        m.action = Marker.ADD
        m.type = Marker.CYLINDER


        frame1 = kdl.Frame(kdl.Rotation.RPY(0.0, math.pi / 2.0, 0.0), kdl.Vector(0.0, 0.0, 0.0))
        frame2 = kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, lenght / 2.0))
        # First, move, then rotate
        frame  = (frame1 * frame2)
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

    def run(self):
        loop_rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                loop_rate.sleep()

                ray_tf = self.pointing_model.pointing_ray()

                if ray_tf:
                    ray_kdl_frame = transform_to_kdl(ray_tf)
                    self.tf_br.sendTransform(ray_tf)

                    ray_pose = PoseStamped()
                    ray_pose.header = ray_tf.header
                    ray_pose.pose = tfc.toMsg(ray_kdl_frame)
                    self.pub_pointing_ray.publish(ray_pose)

                    ray_origin_kdl_frame = self.frame_from_tf(self.human_frame, self.ray_origin_frame)
                    shape_origin_kdl_frame = self.frame_from_tf(self.human_frame, self.ws_shape.frame_id)

                    if ray_origin_kdl_frame and shape_origin_kdl_frame:
                        pointer_pose = self.get_pointer(self.ws_shape, ray_kdl_frame)

                        if pointer_pose:
                            m_msg = self.create_rviz_marker(self.ws_shape, self.ws_shape.point - copy.deepcopy(shape_origin_kdl_frame.p))

                            if m_msg:
                                self.pub_markers.publish(m_msg)

                            pose_msg = PoseStamped()
                            pose_msg.header = ray_tf.header
                            pose_msg.pose = pointer_pose

                            self.pub_arm_pointer.publish(pose_msg)

                            eyes_pointer = tfc.fromMsg(pointer_pose).p - ray_origin_kdl_frame.p
                            pr_marker_msg = self.create_pointing_ray_marker(self.pointing_model.frame_id, eyes_pointer.Norm())

                            if pr_marker_msg:
                                self.pub_markers.publish(pr_marker_msg)

            except rospy.ROSException, e:
                if e.message == 'ROS time moved backwards':
                    rospy.logwarn('Saw a negative time change, resetting.')

if __name__ == '__main__':
    rospy.init_node('pointing_model', anonymous=False)

    node = PointingNode()
    node.run()
#!/usr/bin/env python

import math
import PyKDL as kdl

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped

class PointingModel:
    ''' Abstract pointing model
    '''
    def __init__(self):
        pass

    def pointing_ray(self):
        pass

class HeadFingerModel(PointingModel):
    def __init__(self, tf_buff, fixed_frame, ray_origin_frame, ray_pass_frame, pointing_frame):
        self._tf_buff = tf_buff
        self._fixed_frame = fixed_frame
        self._ray_origin_frame = ray_origin_frame
        self._ray_pass_frame = ray_pass_frame
        self._pointing_frame = pointing_frame

    def _kdl_to_transform(self, k):
        t = TransformStamped()
        t.transform.translation.x = k.p.x()
        t.transform.translation.y = k.p.y()
        t.transform.translation.z = k.p.z()
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = k.M.GetQuaternion()

        return t

    def _frame_from_direction(self, orig_v, dir_v):
        u = kdl.Vector(dir_v.x(), dir_v.y(), dir_v.z())
        u.Normalize()

        yaw = kdl.Rotation.RPY(0.0, 0.0, math.atan2(u.y(), u.x()))
        # Convention: pointing ray is along X-axis
        new_x = yaw * kdl.Vector(1.0, 0.0, 0.0)
        pitch = kdl.Rotation.RPY(0.0, math.atan2(-u.z(), kdl.dot(u, new_x)), 0.0)

        frame = kdl.Frame(yaw * pitch, orig_v)

        return frame

    @property
    def frame_id(self):
        return self._pointing_frame

    def pointing_ray(self):
        ''' Find the pointing ray
        '''
        try:
            origin_tf = self._tf_buff.lookup_transform(self._fixed_frame, self._ray_origin_frame, rospy.Time())
            pass_tf = self._tf_buff.lookup_transform(self._fixed_frame, self._ray_pass_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
            rospy.logerr_throttle(5.0, e.message)
            return None

        pos_origin = kdl.Vector(origin_tf.transform.translation.x,
                      origin_tf.transform.translation.y,
                      origin_tf.transform.translation.z)

        pos_pass = kdl.Vector(pass_tf.transform.translation.x,
                    pass_tf.transform.translation.y,
                    pass_tf.transform.translation.z)

        direction = (pos_pass - pos_origin)

        frame = self._frame_from_direction(pos_origin, direction)

        direction_tf = self._kdl_to_transform(frame)
        direction_tf.header = origin_tf.header
        direction_tf.child_frame_id = self._pointing_frame

        return direction_tf
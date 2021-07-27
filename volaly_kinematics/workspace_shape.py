import sys
import math
import PyKDL as kdl

from geometry_msgs.msg import Pose


class WorkspaceShape(object):
    def __init__(self, frame_id):
        self._frame_id = frame_id

    @property
    def frame_id(self):
        return self._frame_id

    def intersect(self):
        pass

    def toMsg(self, f):
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

    def ray_to_location(self, ray_dir_frame, **kwargs):
        pointer_frame = self.intersect(ray_dir_frame, **kwargs)

        pointer_pose = None

        if pointer_frame:
            pointer_pose = self.toMsg(pointer_frame)

        return pointer_pose


class Plane(WorkspaceShape):
    def __init__(self, frame_id, normal=kdl.Vector(0.0, 0.0, 1.0), point=None):
        super(Plane, self).__init__(frame_id)

        self._normal = normal
        self._point = point

    @property
    def normal(self):
        return self._normal

    @normal.setter
    def normal(self, val):
        self._normal = val

    @property
    def point(self):
        return self._point

    @point.setter
    def point(self, val):
        self._point = val

    def intersect(self, ray_dir_frame, **kwargs):
        if "normal" in kwargs and kwargs["normal"]:
            normal = kwargs["normal"]
        else:
            normal = self._normal

        if "point" in kwargs and kwargs["point"]:
            point = kwargs["point"]
        else:
            point = self._point

        if not (normal and point):
            return None

        u = ray_dir_frame.p - ray_dir_frame * kdl.Vector(1.0, 0.0, 0.0)

        # normal.Normalize()
        n = normal
        w = ray_dir_frame.p - point  # - n * self._distance

        D = kdl.dot(n, u)
        N = -kdl.dot(n, w)

        # Check if the vector is parallel to the plane
        if math.fabs(D) > sys.float_info.epsilon:
            sI = N / D

            # Is the intersection on the back?
            if sI >= 0.0:
                return None

            # Point on the plane
            p = ray_dir_frame.p + sI * u

            yaw = math.atan2(p.y(), p.x())

            # Turn along the pointed direction in horizontal plane
            q = kdl.Rotation.RPY(0.0, 0.0, yaw)

            return kdl.Frame(q, p)

        return None


class Cylinder(WorkspaceShape):
    def __init__(self, frame_id, axis=kdl.Vector(0.0, 0.0, 1.0), point=None):
        super(Cylinder, self).__init__(frame_id)

        self._axis = axis
        self._point = point

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, val):
        self._axis = val

    @property
    def point(self):
        return self._point

    @point.setter
    def point(self, val):
        self._point = val

    def intersect(self, ray_dir_frame, **kwargs):
        if "axis" in kwargs and kwargs["axis"]:
            axis = kwargs["axis"]
        else:
            axis = self._axis

        if "point" in kwargs and kwargs["point"]:
            point = kwargs["point"]
        else:
            point = self._point

        if not (axis and point):
            return None

        u = ray_dir_frame.p - ray_dir_frame * kdl.Vector(1.0, 0.0, 0.0)
        n = axis

        ang = (u * n).Norm()

        # Check if the vector is parallel to the cylinder axis
        if ang > sys.float_info.epsilon:
            # Calculate cylinder radius, but ignore z-coordinate
            r = kdl.Vector(point.x(), point.y(), 0.0).Norm()

            roll, pitch, yaw = ray_dir_frame.M.GetRPY()

            x = r * math.cos(yaw)
            y = r * math.sin(yaw)
            z = -r * math.tan(pitch)

            # Point on the cylinder
            p = ray_dir_frame.p + kdl.Vector(x, y, z)

            # Turn along the pointed direction in horizontal plane
            q = kdl.Rotation.RPY(0.0, 0.0, yaw)

            return kdl.Frame(q, p)

        return None


class Sphere(WorkspaceShape):
    def __init__(self, frame_id, point=None):
        super(Sphere, self).__init__(frame_id)

        self._point = point

    @property
    def point(self):
        return self._point

    @point.setter
    def point(self, val):
        self._point = val

    def intersect(self, ray_dir_frame, **kwargs):
        if "point" in kwargs and kwargs["point"]:
            point = kwargs["point"]
        else:
            point = self._point

        if not point:
            return None

        # Calculate sphere radius
        r = point.Norm()

        # Point on the sphere
        p = ray_dir_frame * kdl.Vector(r, 0.0, 0.0)

        roll, pitch, yaw = ray_dir_frame.M.GetRPY()

        # Turn along the pointed direction in horizontal plane
        q = kdl.Rotation.RPY(0.0, 0.0, yaw)

        return kdl.Frame(q, p)

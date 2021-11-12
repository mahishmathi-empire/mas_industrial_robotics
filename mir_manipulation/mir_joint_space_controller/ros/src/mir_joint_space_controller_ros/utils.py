from __future__ import print_function

import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from brics_actuator.msg import JointVelocities, JointValue

class Utils(object):

    """Utility functions used for joint space controller"""

    @staticmethod
    def get_shortest_angle(angle1, angle2):
        """Compute the angular distance between two angles (in radians)

        :angle1: float
        :angle2: float
        :returns: float
        """
        return math.atan2(math.sin(angle1 - angle2), math.cos(angle1 - angle2))

    @staticmethod
    def get_reverse_angle(angle):
        """Compute the angle facing opposite of given angle and ensures that the 
        returned angle is between pi and -pi
        ASSUMPTION: angle is always between pi and -pi

        :angle: float
        :returns: float
        """
        reverse_angle = angle - math.pi
        if reverse_angle < -math.pi:
            reverse_angle = reverse_angle + (2 * math.pi)
        return reverse_angle

    @staticmethod
    def clip(value, max_allowed=1.0, min_allowed=-1.0):
        """Clip the provided value to be between the given range

        :value: float
        :max_allowed: float
        :min_allowed: float
        :returns: float

        """
        return min(max_allowed, max(min_allowed, value))

    @staticmethod
    def signed_clip(value, max_allowed=1.0, min_allowed=0.1):
        """Clip the provided value to be between the given range while
        maintaining sign

        :value: float
        :max_allowed: float
        :min_allowed: float
        :returns: float

        """
        sign = 1.0 if value > 0.0 else -1.0
        return sign * min(max_allowed, max(min_allowed, abs(value)))

    @staticmethod
    def get_joint_msg_from_joint_angles(joint_angles=None):
        """
        Converts a list of joint angles to a JointPositions message

        :joint_angles: list(float) or None
        :returns: brics_actuator.JointVelocities

        """
        if joint_angles is None or len(joint_angles) != 5:
            joint_angles = [0.0]*5
        joint_msg = JointVelocities()
        for i, joint_angle in enumerate(joint_angles):
            joint_value_msg = JointValue()
            joint_value_msg.joint_uri = 'arm_joint_' + str(i+1)
            joint_value_msg.unit = 's^-1 rad'
            joint_value_msg.value = joint_angle
            joint_msg.velocities.append(joint_value_msg)
        return joint_msg

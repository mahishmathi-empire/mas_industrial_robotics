from __future__ import print_function

import copy
import math
import rospy

from std_msgs.msg import Empty, String
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, PointStamped
from brics_actuator.msg import JointVelocities
from mir_pregrasp_planning_ros.kinematics import Kinematics

from utils import Utils

class JointSpaceController(object):

    """Control youbot arm using joint velocities"""

    def __init__(self):
        # Class variables
        # self._goal_joint_angles = None
        self._goal_joint_angles = [2.16420, 1.13446, -2.5481, 1.78896, 2.93075]
        # self._goal_joint_angles = [3.816, 0.0875, -1.5410, 3.3455, 1.3626]
        self._curr_joint_angles = None
        self._curr_joint_vel = None
        self._kinematics = Kinematics(tip='gripper_static_grasp_link')
        self._num_of_angles = 5

        # tolerances
        self.goal_joint_tolerance = rospy.get_param('~joint_tolerance', 0.1)

        # controller params
        self.p_gain = rospy.get_param('~p_gain', 2.0)
        self.d_gain = rospy.get_param('~d_gain', 1.0)
        self.max_joint_vel = rospy.get_param('~max_joint_vel', [1.5, 1.2, 2.5, 4.0, 3.0])
        # self.max_joint_vel = rospy.get_param('~max_joint_vel', [1.0, 1.0, 1.0, 1.0, 1.0])
        self.min_joint_vel = rospy.get_param('~min_joint_vel', [0.005, 0.005, 0.005, 0.005, 0.005])
        self.max_joint_acc = rospy.get_param('~max_joint_acc', [1.0, 1.0, 2.0, 3.0, 2.0])
        # self.max_joint_acc = rospy.get_param('~max_joint_acc', [1.0, 1.0, 1.0, 1.0, 1.0])
        self.control_rate = rospy.get_param('~control_rate', 5.0)
        self.control_sample_time = 1.0/self.control_rate
        self.prev_error = [0.0]*self._num_of_angles

        # Publishers
        self._joint_vel_pub = rospy.Publisher('~joint_vel', JointVelocities, queue_size=1)

        # Subscribers
        # goal_pose_sub = rospy.Subscriber('~goal', PoseStamped, self.goal_cb)
        joint_value_sub = rospy.Subscriber('~joint_states', JointState, self.joint_states_cb)
        cancel_goal_sub = rospy.Subscriber('~cancel', Empty, self.cancel_current_goal)
        joy_sub = rospy.Subscriber('~joy', Joy, self.joy_cb)

        rospy.sleep(0.2)
        rospy.loginfo('Initialised')

    def run_once(self):
        """
        Main event loop
        """
        print()
        print("inside run_once")
        print(self._curr_joint_angles)
        print(self._curr_joint_vel)
        if self._goal_joint_angles is None or not self._is_within_limits(self._goal_joint_angles):
            return

        if self._curr_joint_angles is None:
            rospy.logwarn('Current joint angles are not available')
            return

        error = [0.0]*self._num_of_angles
        vel = [0.0]*self._num_of_angles
        for i in range(self._num_of_angles):
            error[i] = self._goal_joint_angles[i] - self._curr_joint_angles[i]
            if abs(error[i]) < self.goal_joint_tolerance:
                error[i] = 0.0
                continue
            raw_vel = (self.p_gain * error[i]) + (self.d_gain * (error[i]-self.prev_error[i])/self.control_sample_time)
            raw_clipped_vel = Utils.signed_clip(raw_vel, self.max_joint_vel[i], self.min_joint_vel[i])
            req_acc = (raw_vel - self._curr_joint_vel[i])/self.control_sample_time
            acc = Utils.clip(req_acc, self.max_joint_acc[i], -self.max_joint_acc[i])
            vel[i] = self._curr_joint_vel[i] + (acc * self.control_sample_time)
        self.prev_error = error

        print("error:", error)
        print("vel:", vel)

        if error == [0.0]*5:
            print("REACHED GOAL")
            self._goal_joint_angles = None
            self._reset_state()
            return
        self._joint_vel_pub.publish(Utils.get_joint_msg_from_joint_angles(vel))

    def joint_states_cb(self, msg):
        if "arm_joint" in msg.name[0]:
            self._curr_joint_angles = msg.position
            self._curr_joint_vel = msg.velocity

    def joy_cb(self, msg):
        if msg.buttons[5] == 1:
            rospy.logwarn('PREEMPTING (joypad interrupt)')
            self._reset_state()

    def _reset_state(self):
        self._goal_joint_angles = None
        self.publish_zero_vel()

    def cancel_current_goal(self, msg):
        rospy.logwarn('PREEMPTING (cancelled goal)')
        self._reset_state()

    def publish_zero_vel(self):
        self.last_applied_vel_x = 0.0
        self._joint_vel_pub.publish(Utils.get_joint_msg_from_joint_angles())

    def _is_within_limits(self, joint_angles):
        for i in range(self._num_of_angles):
            if joint_angles[i] < self._kinematics.lower_limit[i] or\
                    joint_angles[i] > self._kinematics.upper_limit[i]:
                return False
        return True

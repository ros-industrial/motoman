#!/usr/bin/env python

# TODO: add license

# A simple example showing the use of an Action client to request execution of
# a complete JointTrajectory.
#
# Note: joint names must match those used in the urdf, or the driver will
#       refuse to accept the goal.

# https://github.com/ros-industrial/motoman/compare/kinetic-devel...gavanderhoorn:actionclient_example

import rospy
import actionlib
import time
import math

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



def main():
    rospy.init_node('simple_trajectory_action_client')

    # create client and make sure it's available
    client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)
    rospy.loginfo('Waiting for driver\'s action server to become available ..')
    client.wait_for_server()
    rospy.loginfo('Connected to trajectory action server')

    # setup simple goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()

    # this should correspond to the joint names of the robot being used (ie:
    # the joint names as specific in the urdf). The list specified here contains
    # the default Motoman joints.
    # NOTE: order matters here
    goal.trajectory.joint_names = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']

    # motoman_driver only accepts goals with trajectories that start at the
    # current robot state, so retrieve that and use it as the first point in
    # the trajectory
    robot_joint_states = rospy.wait_for_message('joint_states', JointState)

    # make sure the state we get contains the same joints (both amount and names)
    if set(goal.trajectory.joint_names) != set(robot_joint_states.name):
        rospy.logfatal("Mismatch between joints specified and seen in current "
            "JointState. Expected: '{}', got: '{}'. Cannot continue.".format(
                ', '.join(robot_joint_states.name),
                ', '.join(goal.trajectory.joint_names)))
        sys.exit(1)

    # create the trajectory: it will consist of three points:
    #
    #   1. T= 0: current pose
    #   2. T= 5: T joint rotated -20 degrees
    #   3. T=10: T joint rotated +20 degrees

    q0 = robot_joint_states.position
    q1 = list(q0)
    q2 = list(q0)

    # Assume the last joint is T
    q1[-1] -= math.radians(20)

    # Make the robot come to a complete stop at each trajectory point (ie:
    # zero target velocity).
    qdot = [0.0] * len(goal.trajectory.joint_names)

    # add points
    goal.trajectory.points.append(JointTrajectoryPoint(positions=q0,
        velocities=qdot, time_from_start=rospy.Duration( 0.0)))

    goal.trajectory.points.append(JointTrajectoryPoint(positions=q1,
        velocities=qdot, time_from_start=rospy.Duration( 5.0)))

    goal.trajectory.points.append(JointTrajectoryPoint(positions=q2,
        velocities=qdot, time_from_start=rospy.Duration(10.0)))

    # goal constructed, submit it for execution
    rospy.loginfo("Submitting goal ..")
    client.send_goal(goal)
    rospy.loginfo("Waiting for completion ..")
    client.wait_for_result()
    rospy.loginfo('Done.')


if __name__ == '__main__':
    main()
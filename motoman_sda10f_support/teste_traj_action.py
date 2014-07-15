#!/usr/bin/env python


import roslib; roslib.load_manifest('motoman_sda10f_support')
import rospy
import math
import sys
import string
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from sensor_msgs.msg import JointState
from actionlib import simple_action_client
from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int64

import random

class teste_traj():
    def __init__(self):

        #initializing ROS
        rospy.init_node('teste_traj_action')

        self.alternate_value = 0.0

    def teste_traj_action(self):
    

        #self.alternate_value = (self.alternate_value+0.4)%1.2
        self.alternate_value = random.uniform(-1.2, 1.2)
        
        ## r1
        msg_r1 = rospy.wait_for_message("/sda10f/sda10f_r1_controller/joint_states", JointState, 5.0)
        sda10f_r1_client = actionlib.SimpleActionClient('/sda10f/sda10f_r1_controller/joint_trajectory_action', FollowJointTrajectoryAction)
        sda10f_r1_client.wait_for_server()

        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        # Populates trajectory with joint names.
        goal.trajectory.joint_names = ['joint_r1_s', 'joint_r1_l', 'joint_r1_e', 'joint_r1_u', 'joint_r1_r', 'joint_r1_b', 'joint_r1_t']

        # First trajectory point
        # Positions
        ind = 0
        #print goal.trajectory.points
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        goal.trajectory.points = [point1, point2]
        
        point1.positions = msg_r1.position
        point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point1.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point1.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        goal.trajectory.points[ind] = point1
        goal.trajectory.points[ind].time_from_start = rospy.Duration(0.0)

        # Second trajectory point
        # Positions
        ind += 1
        point2.positions = [self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value]
        point2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point2.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point2.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        goal.trajectory.points[ind] = point2
        goal.trajectory.points[ind].time_from_start = rospy.Duration(2.0)
    
        goal.trajectory.header.stamp = rospy.Time.now()
        
        sda10f_r1_client.send_goal(goal)
        
        rospy.sleep(0.4)
        
        ## r2
        
        sda10f_r2_client = actionlib.SimpleActionClient('/sda10f/sda10f_r2_controller/joint_trajectory_action', FollowJointTrajectoryAction)
        sda10f_r2_client.wait_for_server()
        msg_r2 = rospy.wait_for_message("/sda10f/sda10f_r2_controller/joint_states", JointState, 5.0)
        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        # Populates trajectory with joint names.
        goal.trajectory.joint_names = ['joint_r2_s', 'joint_r2_l', 'joint_r2_e', 'joint_r2_u', 'joint_r2_r', 'joint_r2_b', 'joint_r2_t']

        # First trajectory point
        # Positions
        ind = 0
        #print goal.trajectory.points
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        goal.trajectory.points = [point1, point2]
        
        point1.positions = msg_r2.position
        point1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point1.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point1.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        goal.trajectory.points[ind] = point1
        goal.trajectory.points[ind].time_from_start = rospy.Duration(0.0)

        # Second trajectory point
        # Positions
        ind += 1
        point2.positions = [self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value]
        point2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point2.accelerations = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point2.effort = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        goal.trajectory.points[ind] = point2
        goal.trajectory.points[ind].time_from_start = rospy.Duration(2.0)
    
        goal.trajectory.header.stamp = rospy.Time.now()
        
        sda10f_r2_client.send_goal(goal)
        
        rospy.sleep(0.4)
        # b1 and b2#
        
        ## b1
        
        msg_b1 = rospy.wait_for_message("/sda10f/sda10f_b1_controller/joint_states", JointState, 5.0)
        msg_b2 = rospy.wait_for_message("/sda10f/sda10f_b2_controller/joint_states", JointState, 5.0)
        
        sda10f_b1_client = actionlib.SimpleActionClient('/sda10f/sda10f_b1_controller/joint_trajectory_action', FollowJointTrajectoryAction)
        sda10f_b1_client.wait_for_server()

        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        # Populates trajectory with joint names.
        goal.trajectory.joint_names = ['joint_b1']

        # First trajectory point
        # Positions
        ind = 0
        #print goal.trajectory.points
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        goal.trajectory.points = [point1, point2]

        point1.positions = msg_b1.position
        point1.velocities = [0.0]
        point1.accelerations = [0.0]
        point1.effort = [0.0]
        goal.trajectory.points[ind] = point1
        goal.trajectory.points[ind].time_from_start = rospy.Duration(0.0)

        # Second trajectory point
        # Positions
        ind += 1
        point2.positions = [self.alternate_value]
        point2.velocities = [0.0]
        point2.accelerations = [0.0]
        point2.effort = [0.0]
        goal.trajectory.points[ind] = point2
        goal.trajectory.points[ind].time_from_start = rospy.Duration(2.0)
    
        goal.trajectory.header.stamp = rospy.Time.now()
        
        sda10f_b1_client.send_goal(goal)
        
        rospy.sleep(0.4)
        # B2
        sda10f_b2_client = actionlib.SimpleActionClient('/sda10f/sda10f_b2_controller/joint_trajectory_action', FollowJointTrajectoryAction)
        sda10f_b2_client.wait_for_server()

        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()

        # Populates trajectory with joint names.
        goal.trajectory.joint_names = ['joint_b2']

        # First trajectory point
        # Positions
        ind = 0
        #print goal.trajectory.points
        point1 = trajectory_msgs.msg.JointTrajectoryPoint()
        point2 = trajectory_msgs.msg.JointTrajectoryPoint()
        goal.trajectory.points = [point1, point2]

        point1.positions = msg_b2.position
        point1.velocities = [0.0]
        point1.accelerations = [0.0]
        point1.effort = [0.0]
        goal.trajectory.points[ind] = point1
        goal.trajectory.points[ind].time_from_start = rospy.Duration(0.0)

        # Second trajectory point
        # Positions
        ind += 1
        point2.positions = [self.alternate_value]
        point2.velocities = [0.0]
        point2.accelerations = [0.0]
        point2.effort = [0.0]
        goal.trajectory.points[ind] = point2
        goal.trajectory.points[ind].time_from_start = rospy.Duration(2.0)
    
        goal.trajectory.header.stamp = rospy.Time.now()
        
        sda10f_b2_client.send_goal(goal)
        
        rospy.sleep(2.0)

        


if __name__=='__main__':
    rep = teste_traj()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rep.teste_traj_action()
        r.sleep()



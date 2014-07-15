#!/usr/bin/env python
import roslib; roslib.load_manifest('motoman_sda10f_support')

import sys

import rospy
from control_msgs.msg import *
from sensor_msgs.msg import *
from industrial_msgs.msg import DynamicJointTrajectory, DynamicJointsGroup, DynamicJointPoint

import random

class FakeJoint:
    def __init__(self):
        self._pub_controller_state_r2 = rospy.Publisher('sda10f/sda10f_r2_controller/joint_path_command', DynamicJointTrajectory)
       
        self._pub_controller_state_r1 = rospy.Publisher('sda10f/sda10f_r1_controller/joint_path_command', DynamicJointTrajectory)
        
        self._pub_controller_state_b1 = rospy.Publisher('sda10f/sda10f_b1_controller/joint_path_command', DynamicJointTrajectory)
       
        self._pub_controller_state_b2 = rospy.Publisher('sda10f/sda10f_b2_controller/joint_path_command', DynamicJointTrajectory)
        
        self.alternate_value = 0.0

        self._controller_state = DynamicJointTrajectory()

        
    def publish_controller_state(self):
        
        self.alternate_value = random.uniform(-1.2, 1.2)
        #r1 ---------------------------------------
        #msg = rospy.wait_for_message("sda10f/sda10f_r1_controller/joint_states", JointState, 5.0)
        msg = rospy.wait_for_message("sda10f/sda10f_r2_controller/joint_states", JointState, 5.0)
        #msg = rospy.wait_for_message("sda10f/sda10f_b1_controller/joint_states", JointState, 5.0)
        #msg = rospy.wait_for_message("sda10f/sda10f_b2_controller/joint_states", JointState, 5.0)
        
        
        dj_group = DynamicJointsGroup()
        dj_group.group_number = 1
        dj_group.num_joints = 7
        dj_group.positions = [self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value]
        dj_group.velocities = [0,0,0,0,0,0,0]
        dj_group.accelerations = [0,0,0,0,0,0,0]
        dj_group.effort = [0,0,0,0,0,0,0]
        dj_group.time_from_start = rospy.Duration(2.0)
        
        dj_point = DynamicJointPoint()
        dj_point.groups.append(dj_group)
        dj_point.num_groups = 1
        
        dj_group_end = DynamicJointsGroup()
        dj_group_end.group_number = 1
        dj_group_end.num_joints = 7
        dj_group_end.positions = msg.position
        dj_group_end.velocities = [0,0,0,0,0,0,0] 
        dj_group_end.accelerations = [0,0,0,0,0,0,0]
        dj_group_end.effort = [0,0,0,0,0,0,0] 
        dj_group_end.time_from_start = rospy.Duration(0.0)
        
        dj_point_end = DynamicJointPoint()
        dj_point_end.groups.append(dj_group_end)
        dj_point_end.num_groups = 1
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_r2_s', 'joint_r2_l', 'joint_r2_e', 'joint_r2_u', 'joint_r2_r', 'joint_r2_b', 'joint_r2_t']
        self._controller_state.points = [dj_point_end, dj_point]
        
        self._pub_controller_state_r2.publish(self._controller_state)
        rospy.sleep(0.4)
        #r2 ---------------------------------------
        msg = rospy.wait_for_message("sda10f/sda10f_r1_controller/joint_states", JointState, 5.0)  
        
        
        dj_group = DynamicJointsGroup()
        dj_group.group_number = 0
        dj_group.num_joints = 7
        #dj_group.positions = [self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value,self.alternate_value]
        dj_group.positions = [0,0,0,0,0,0,0]
        dj_group.velocities = [0,0,0,0,0,0,0]
        dj_group.accelerations = [0,0,0,0,0,0,0]
        dj_group.time_from_start = rospy.Duration(2.0)
        
        dj_point = DynamicJointPoint()
        dj_point.groups.append(dj_group)
        dj_point.num_groups = 1
        
        dj_group_end = DynamicJointsGroup()
        dj_group_end.group_number = 0
        dj_group_end.num_joints = 7
        dj_group_end.positions = msg.position
        dj_group_end.velocities = [0,0,0,0,0,0,0] 
        dj_group_end.accelerations = [0,0,0,0,0,0,0] 
        dj_group_end.time_from_start = rospy.Duration(0.0)
        
        dj_point_end = DynamicJointPoint()
        dj_point_end.groups.append(dj_group_end)
        dj_point_end.num_groups = 1
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_r1_s', 'joint_r1_l', 'joint_r1_e', 'joint_r1_u', 'joint_r1_r', 'joint_r1_b', 'joint_r1_t']
        self._controller_state.points = [dj_point_end, dj_point]
        
        self._pub_controller_state_r1.publish(self._controller_state)
        
        rospy.sleep(0.4)
        #b1 and b2 ---------------------------------------
        msg_b1 = rospy.wait_for_message("sda10f/sda10f_b1_controller/joint_states", JointState, 5.0) 
        
        msg_b2 = rospy.wait_for_message("sda10f/sda10f_b2_controller/joint_states", JointState, 5.0) 
        
        
        dj_group = DynamicJointsGroup()
        dj_group.group_number = 2
        dj_group.num_joints = 1
        dj_group.positions = [self.alternate_value]
        dj_group.velocities = [0]
        dj_group.accelerations = [0]
        dj_group.time_from_start = rospy.Duration(2.0)
        
        dj_point = DynamicJointPoint()
        dj_point.groups.append(dj_group)
        dj_point.num_groups = 1
        
        dj_group_end = DynamicJointsGroup()
        dj_group_end.group_number = 2
        dj_group_end.num_joints = 1
        dj_group_end.positions = msg_b1.position
        dj_group_end.velocities = [0] 
        dj_group_end.accelerations = [0] 
        dj_group_end.time_from_start = rospy.Duration(0.0)
        
        dj_point_end = DynamicJointPoint()
        dj_point_end.groups.append(dj_group_end)
        dj_point_end.num_groups = 1
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_b1']
        self._controller_state.points = [dj_point_end, dj_point]
        
        self._pub_controller_state_b1.publish(self._controller_state)
        
        rospy.sleep(0.4)
        
        dj_group = DynamicJointsGroup()
        dj_group.group_number = 3
        dj_group.num_joints = 1
        dj_group.positions = [self.alternate_value]
        dj_group.velocities = [0]
        dj_group.accelerations = [0]
        dj_group.effort = [0]
        dj_group.time_from_start = rospy.Duration(2.0)
        
        dj_point = DynamicJointPoint()
        dj_point.groups.append(dj_group)
        dj_point.num_groups = 1
        
        dj_group_end = DynamicJointsGroup()
        dj_group_end.group_number = 3
        dj_group_end.num_joints = 1
        dj_group_end.positions = msg_b2.position
        dj_group_end.velocities = [0] 
        dj_group_end.accelerations = [0] 
        dj_group_end.time_from_start = rospy.Duration(0.0)
        
        dj_point_end = DynamicJointPoint()
        dj_point_end.groups.append(dj_group_end)
        dj_point_end.num_groups = 1
        
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_b2']
        self._controller_state.points = [dj_point_end, dj_point]
        
        self._pub_controller_state_b2.publish(self._controller_state)
        
        rospy.sleep(2.0)

if __name__ == '__main__':
    rospy.init_node('pg70_fake_joint')
    print "starting fake joint"
    server = FakeJoint()
    print "fake joint started"
    r = rospy.Rate(2) # 10hz
    server.publish_controller_state()
    while not rospy.is_shutdown():
        server.publish_controller_state()
        r.sleep()   
    
    



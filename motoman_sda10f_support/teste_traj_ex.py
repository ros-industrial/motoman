#!/usr/bin/env python
import roslib; roslib.load_manifest('motoman_sda10f_support')

import sys

import rospy
from control_msgs.msg import *
from sensor_msgs.msg import *
from industrial_msgs.msg import DynamicJointTrajectory, DynamicJointsGroup, DynamicJointPoint

class teste_traj_ex:
    def __init__(self):
        self._pub_controller_state = rospy.Publisher('joint_path_command', DynamicJointTrajectory)
        
        self.alternate_value = 0.0

        
    def publish_controller_state(self):
        
        self.alternate_value = (self.alternate_value+0.5)%1.0
        #r1 ---------------------------------------
        msg0 = rospy.wait_for_message("sda10f/sda10f_r1_controller/joint_states", JointState, 5.0)
        msg1 = rospy.wait_for_message("sda10f/sda10f_r2_controller/joint_states", JointState, 5.0)
        msg2 = rospy.wait_for_message("sda10f/sda10f_b1_controller/joint_states", JointState, 5.0)
        msg3 = rospy.wait_for_message("sda10f/sda10f_b2_controller/joint_states", JointState, 5.0)
        
        
        
        dj_r1 = DynamicJointsGroup()
        dj_r1.group_number = 0
        dj_r1.num_joints = 7
        dj_r1.positions = msg0.position
        dj_r1.velocities = [0,0,0,0,0,0,0]
        dj_r1.accelerations = [0,0,0,0,0,0,0]
        dj_r1.effort = [0,0,0,0,0,0,0]
        dj_r1.time_from_start = rospy.Duration(0.0)
        
       
        dj_r2 = DynamicJointsGroup()
        dj_r2.group_number = 1
        dj_r2.num_joints = 7
        dj_r2.positions = msg1.position
        dj_r2.velocities = [0,0,0,0,0,0,0] 
        dj_r2.accelerations = [0,0,0,0,0,0,0] 
        dj_r2.effort = [0,0,0,0,0,0,0] 
        dj_r2.time_from_start = rospy.Duration(0.0)
        
        dj_b1 = DynamicJointsGroup()
        dj_b1.group_number = 2
        dj_b1.num_joints = 1
        dj_b1.positions = msg2.position
        dj_b1.velocities = [0]
        dj_b1.accelerations = [0]
        dj_b1.effort = [0]
        dj_b1.time_from_start = rospy.Duration(0.0)
        
        dj_b2 = DynamicJointsGroup()
        dj_b2.group_number = 3
        dj_b2.num_joints = 1
        dj_b2.positions = msg3.position
        dj_b2.velocities = [0] 
        dj_b2.accelerations = [0] 
        dj_b2.time_from_start = rospy.Duration(0.0)
        
        dj_p = DynamicJointPoint()
        dj_p.groups = [dj_r1, dj_r2, dj_b1, dj_b2]
        dj_p.num_groups = 4
        
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_r1_s', 'joint_r1_l', 'joint_r1_e', 'joint_r1_u', 'joint_r1_r', 'joint_r1_b', 'joint_r1_t','joint_r2_s', 'joint_r2_l', 'joint_r2_e', 'joint_r2_u', 'joint_r2_r', 'joint_r2_b', 'joint_r2_t', 'joint_b1', 'joint_b2']
        self._controller_state.points = [dj_p]
        
        self._pub_controller_state.publish(self._controller_state)
        
        rospy.sleep(2.0)
        
        # second point
        
        dj_r1 = DynamicJointsGroup()
        dj_r1.group_number = 0
        dj_r1.num_joints = 7
        dj_r1.positions = [0,0,self.alternate_value,0,0,0,0]
        dj_r1.velocities = [0,0,0,0,0,0,0]
        dj_r1.accelerations = [0,0,0,0,0,0,0]
        dj_r1.effort = [0,0,0,0,0,0,0]
        dj_r1.time_from_start = rospy.Duration(2.0)
        
        dj_r2 = DynamicJointsGroup()
        dj_r2.group_number = 1
        dj_r2.num_joints = 7
        dj_r2.positions = [self.alternate_value,0,0,0,0,0,0]
        dj_r2.velocities = [0,0,0,0,0,0,0] 
        dj_r2.accelerations = [0,0,0,0,0,0,0] 
        dj_r2.effort = [0,0,0,0,0,0,0] 
        dj_r2.time_from_start = rospy.Duration(4.0)
        
        dj_b1 = DynamicJointsGroup()
        dj_b1.group_number = 2
        dj_b1.num_joints = 1
        dj_b1.positions = [self.alternate_value]
        dj_b1.velocities = [0]
        dj_b1.accelerations = [0]
        dj_b1.effort = [0]
        dj_b1.time_from_start = rospy.Duration(6.0)
        
        dj_b2 = DynamicJointsGroup()
        dj_b2.group_number = 3
        dj_b2.num_joints = 1
        dj_b2.positions = [self.alternate_value]
        dj_b2.velocities = [0] 
        dj_b2.accelerations = [0] 
        dj_b2.time_from_start = rospy.Duration(8.0)
        
        dj_p = DynamicJointPoint()
        dj_p.groups = [dj_r1, dj_r2, dj_b1, dj_b2]
        dj_p.num_groups = 4
        
        self._controller_state = DynamicJointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_r1_s', 'joint_r1_l', 'joint_r1_e', 'joint_r1_u', 'joint_r1_r', 'joint_r1_b', 'joint_r1_t','joint_r2_s', 'joint_r2_l', 'joint_r2_e', 'joint_r2_u', 'joint_r2_r', 'joint_r2_b', 'joint_r2_t', 'joint_b1', 'joint_b2']
        self._controller_state.points = [dj_p]
        
        self._pub_controller_state.publish(self._controller_state)
        
        rospy.sleep(10.0)

if __name__ == '__main__':
    rospy.init_node('teste_traj_ex')

    server = teste_traj_ex()

    r = rospy.Rate(2) # 10hz
    
    server.publish_controller_state()
    
    while not rospy.is_shutdown():
        server.publish_controller_state()
        r.sleep()   
    
    



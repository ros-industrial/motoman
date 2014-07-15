#!/usr/bin/env python
import roslib; roslib.load_manifest('motoman_sda10f_support')

import sys

from sensor_msgs.msg import JointState
import rospy
from control_msgs.msg import *
from sensor_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FakeJoint:
    def __init__(self):
        self._pub_controller_state = rospy.Publisher('joint_path_command', JointTrajectory)
        
        dj_point = JointTrajectoryPoint()
        dj_point.positions = [0,0,0,0,0,0,0]
        dj_point.velocities = [0,0,0,0,0,0,0]   
        dj_point.time_from_start = rospy.Duration(5.0)
        
        #msg = rospy.wait_for_message("sda10f/sda10f_r1_controller/joint_states", JointState, 5.0)
        msg = rospy.wait_for_message("joint_states", JointState, 5.0)
        #msg = rospy.wait_for_message("sda10f/sda10f_b1_controller/joint_states", JointState, 5.0)
        #msg = rospy.wait_for_message("sda10f/sda10f_b2_controller/joint_states", JointState, 5.0)
        
        print msg
        
        dj_point_end = JointTrajectoryPoint()

        dj_point_end.positions = msg.position
        dj_point_end.velocities = [0,0,0,0,0,0,0] 
        dj_point_end.time_from_start = rospy.Duration(0.0)
        
        self._controller_state = JointTrajectory()
        self._controller_state.header.stamp = rospy.Time.now()
        self._controller_state.joint_names = ['joint_s', 'joint_l', 'joint_e', 'joint_u', 'joint_r', 'joint_b', 'joint_t']
        #self._controller_state.joint_names = ['joint_r1_s', 'joint_r2_l', 'joint_r2_e', 'joint_r2_u', 'joint_r2_r', 'joint_r2_b', 'joint_r2_t']
        self._controller_state.points = [dj_point_end,dj_point]
        
        self._pub_controller_state.publish(self._controller_state)  
        
    def publish_controller_state(self):
        self._controller_state.header.stamp = rospy.Time.now()
        self._pub_controller_state.publish(self._controller_state)

if __name__ == '__main__':
    rospy.init_node('pg70_fake_joint')
    print "starting fake joint"
    server = FakeJoint()
    print "fake joint started"
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        server.publish_controller_state()
        r.sleep()   
    
    



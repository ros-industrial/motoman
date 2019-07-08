#!/usr/bin/python
import rospy

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from motoman_msgs.msg import DynamicJointTrajectory
from motoman_msgs.msg import DynamicJointPoint
from motoman_msgs.msg import DynamicJointsGroup

from control_msgs.msg import FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState
from industrial_msgs.msg import RobotStatus

import threading

'''
Subscribe to multiple joint_states and feedback_states topics and combine them
into a unified 7 dof message and publish.
Subscribe to a 7dof trajectory_msgs/JointTrajectory message on the topic
joint_path_command and publish two motoman_msgs/DynamicJointtrajectory messages
on the topics /dx200/dx200_[s1,r1]_controller/joint_path_command.
'''

class dynamic_trajectory_converter(object):
  def __init__(self):
    # Get message arrival tolerance to combine separate messages (seconds)
    self.time_tol = rospy.get_param('~time_tol',0.001)

    # Get namespaces from the parameter server
    moto_ns = rospy.get_param('~moto_ns','dx200')
    if moto_ns: # If the namspace isn't empty, add a '/'
      moto_ns = moto_ns + "/"
    dof_ns = rospy.get_param('~7dof_ns','ma1440_d500')
    if dof_ns: # If the namspace isn't empty, add a '/'
      dof_ns = dof_ns + "/"

    # Subscribe to joint_states from robot/simulator
    rospy.Subscriber(moto_ns + 'dx200_r1_controller/joint_states',
                     JointState, self.r1_joint_state_cb)
    rospy.Subscriber(moto_ns + 'dx200_s1_controller/joint_states',
                     JointState, self.s1_joint_state_cb)
    # Publish joint_states to rest of system
    self.joint_states_pub = rospy.Publisher(dof_ns+'joint_states', JointState,
                                           queue_size=1)
    # Subscribe to feedback_states from robot/simulator
    rospy.Subscriber(moto_ns + 'dx200_r1_controller/feedback_states',
                     FollowJointTrajectoryFeedback, self.r1_feedback_state_cb)
    rospy.Subscriber(moto_ns + 'dx200_s1_controller/feedback_states',
                     FollowJointTrajectoryFeedback, self.s1_feedback_state_cb)
    # Subscribe to the robot status
    rospy.Subscriber('robot_status', RobotStatus, self.robot_status_cb)
    # Publish robot staus on a new topic
    self.robot_status_pub = rospy.Publisher(dof_ns+'robot_status', RobotStatus,
                                            queue_size=1)
    # Publish joint_states to rest of system
    self.feedback_states_pub = rospy.Publisher(dof_ns+'feedback_states',
                                              FollowJointTrajectoryFeedback,
                                              queue_size=1)
    # Maintain a memory of the most recent joint states and feedback states
    self.r1_joint_states = JointState()
    self.s1_joint_states = JointState()
    self.r1_feedback_states = FollowJointTrajectoryFeedback()
    self.s1_feedback_states = FollowJointTrajectoryFeedback()

    # Locks to prevent pub_joint_states and pub_feedback_states from running
    # multiple instances at the same time.
    self.joint_states_lock = threading.Lock()
    self.feedback_states_lock = threading.Lock()

    # Subscribe to joint_path_command from system
    rospy.Subscriber(dof_ns + 'joint_path_command', JointTrajectory,
                     self.joint_path_command_cb) # looking at this in rostopic echo /dx200_7d0f/joint_path_command, the matrix goes [b,l,r,s,t,u,x]...
    # Publish joint_path_command to robot/simulator
    self.joint_path_command_pub = rospy.Publisher('/joint_path_command',
                                                  DynamicJointTrajectory,
                                                  queue_size=1)


  # Store the joint state in memory and publish if they are close, temporally
  def r1_joint_state_cb(self,msg):
    self.joint_states_lock.acquire()
    self.r1_joint_states = msg
    self.pub_joint_states()
    self.joint_states_lock.release()

  def s1_joint_state_cb(self,msg):
    self.joint_states_lock.acquire()
    self.s1_joint_states = msg
    self.pub_joint_states()
    self.joint_states_lock.release()

  def pub_joint_states(self):
    # Check to see if two joint state share the same sequence number
    # Should protect against running before both messages have been received
    time_diff = self.r1_joint_states.header.stamp.to_sec() - self.s1_joint_states.header.stamp.to_sec()
    if (self.r1_joint_states.header.seq != self.s1_joint_states.header.seq):
      # If not, get out of the function and wait until it is called again
      rospy.logdebug("joint_states have different seq :(")
      rospy.logdebug('r1 seq = {0}, s1 seq = {1}'.format(self.r1_joint_states.header.seq,self.s1_joint_states.header.seq))
    else:
      rospy.logdebug("joint_states have the same seq :)")
      # Combine into a 7dof message
      joint_states = JointState()
      if (time_diff <= 0) :
        joint_states.header.stamp = self.r1_joint_states.header.stamp
      else:
        joint_states.header.stamp = self.s1_joint_states.header.stamp
      name = ['joint_s','joint_l','joint_u','joint_r','joint_b','joint_t']
      position = self.r1_joint_states.position # + (self.s1_joint_states.position[0], self.s1_joint_states.position[1])
      if (len(self.s1_joint_states.velocity) < 1) :
        velocity = self.r1_joint_states.velocity # + (0.0,)
      else:
        velocity = self.r1_joint_states.velocity # + (self.s1_joint_states.velocity[0], self.s1_joint_states.velocity[1])
      joint_states.name = name
      joint_states.position = position
      joint_states.velocity = velocity
      # And publish to a new topic
      self.joint_states_pub.publish(joint_states)

  # Store the feedback state in memory and publish if they are close, temporally
  def r1_feedback_state_cb(self,msg):
    self.feedback_states_lock.acquire()
    self.r1_feedback_states = msg
    self.pub_feedback_states()
    self.feedback_states_lock.release()

  def s1_feedback_state_cb(self,msg):
    self.feedback_states_lock.acquire()
    self.s1_feedback_states = msg
    self.pub_feedback_states()
    self.feedback_states_lock.release()

  def pub_feedback_states(self):
    # Check to see if two joint state share the same sequence number
    # Should protect against running before both messages have been received
    time_diff = self.r1_feedback_states.header.stamp.to_sec() - self.s1_feedback_states.header.stamp.to_sec()
    if (self.r1_feedback_states.header.seq != self.s1_feedback_states.header.seq):
      # If not, get out of the function and wait until it is called again
      rospy.logdebug("feedback_states have different seq :(")
      rospy.logdebug('r1 seq = {0}, s1 seq = {1}'.format(self.r1_feedback_states.header.seq,self.s1_feedback_states.header.seq))
    else:
      rospy.logdebug("feedback_states have the same seq :)")
      # Combine into a 7dof message
      feedback_states = FollowJointTrajectoryFeedback()
      if (time_diff <= 0) :
        feedback_states.header.stamp = self.r1_feedback_states.header.stamp
      else:
        feedback_states.header.stamp = self.s1_feedback_states.header.stamp
      joint_names = ['joint_s','joint_l','joint_u','joint_r','joint_b','joint_t']
      positions = self.r1_feedback_states.actual.positions # + (self.s1_feedback_states.actual.positions[0], self.s1_feedback_states.actual.positions[1])
      if (len(self.s1_feedback_states.actual.velocities) < 1) :
        velocities = self.r1_feedback_states.actual.velocities + (0.0,)
      else:
        velocities = self.r1_feedback_states.actual.velocities # + (self.s1_feedback_states.actual.velocities[0], self.s1_feedback_states.actual.velocities[1])
      feedback_states.joint_names = joint_names
      feedback_states.actual.positions = positions
      feedback_states.actual.velocities = velocities
      # And publish to a new topic
      self.feedback_states_pub.publish(feedback_states)

  def joint_path_command_cb(self,msg):
    # Divide the 7dof message into two separate groups
    points = ()
    for i, dyn_point in enumerate(msg.points):
      # Group 0 - r1 - welder arm
      group_r1 = DynamicJointsGroup()
      group_r1.group_number = 0
      group_r1.num_joints = 6
      group_r1.valid_fields = 0
      group_r1.positions = (dyn_point.positions[0], dyn_point.positions[1],dyn_point.positions[2],dyn_point.positions[3],dyn_point.positions[4],dyn_point.positions[5])
      # group_r1.positions = (dyn_point.positions[3], dyn_point.positions[1],dyn_point.positions[5],dyn_point.positions[2],dyn_point.positions[0],dyn_point.positions[4]) #dyn_point.positions[:6]
      # group_r1.velocities = (dyn_point.velocities[3],dyn_point.velocities[1],dyn_point.velocities[5],dyn_point.velocities[2],dyn_point.velocities[0],dyn_point.velocities[4])
      group_r1.velocities = (0,0,0,0,0,0)
      # group_r1.accelerations = (dyn_point.accelerations[3],dyn_point.accelerations[1],dyn_point.accelerations[5],dyn_point.accelerations[2],dyn_point.accelerations[0],dyn_point.accelerations[4])
      group_r1.accelerations = (0,0,0,0,0,0)
      group_r1.time_from_start = dyn_point.time_from_start
      # Group 1 - s1 - positioner
      group_s1 = DynamicJointsGroup()
      group_s1.group_number = 1
      #group_s1.num_joints = 6
      group_s1.valid_fields = 0
      #group_s1.positions = dyn_point.positions[6:]+(0,0,0,0,0)
      #group_s1.velocities = dyn_point.velocities[6:]+(0,0,0,0,0)
      #group_s1.accelerations = dyn_point.accelerations[6:]+(0,0,0,0,0)
      group_s1.num_joints = 2
      group_s1.positions = dyn_point.positions[6:]
      group_s1.velocities = dyn_point.velocities[6:]
      group_s1.accelerations = dyn_point.accelerations[6:]
      group_s1.time_from_start = dyn_point.time_from_start
      point = DynamicJointPoint()
      point.num_groups = 2
      point.groups = (group_r1,group_s1)
      points = points + (point,)
    # rospy.loginfo(points)
    dyn_traj = DynamicJointTrajectory()
    dyn_traj.header = msg.header
    joint_names = ['joint_s','joint_l','joint_u','joint_r','joint_b','joint_t']
    dyn_traj.joint_names = joint_names
    dyn_traj.points = points
    # and publish
    self.joint_path_command_pub.publish(dyn_traj)

  def robot_status_cb(self,msg):
    self.robot_status_pub.publish(msg)

  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node('dynamic_trajectory_converter')
  dtj = dynamic_trajectory_converter()
  dtj.spin()
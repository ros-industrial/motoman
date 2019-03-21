#!/usr/bin/python
import rospy
import rospkg

from control_msgs.msg import FollowJointTrajectoryFeedback
from sensor_msgs.msg import JointState

import yaml

class actuation_angle_offsets(object):
  def __init__(self):
    rospy.logdebug("initializing actuation_angle_offsets")
    # Get namespaces from the parameter server
    # robot_side_ns are the values coming directly from the motoman driver
    robot_side_ns = rospy.get_param('~robot_side_ns','dx200_corr')
    if robot_side_ns: # If the namspace isn't empty, add a '/'
      robot_side_ns = robot_side_ns + "/"
    # comp_side_ns are the values to be corrected with the offset
    comp_side_ns = rospy.get_param('~comp_side_ns','')
    if comp_side_ns: # If the namspace isn't empty, add a '/'
      comp_side_ns = comp_side_ns + "/"

    # Subscribe to joint_states from robot/simulator
    rospy.Subscriber(robot_side_ns + 'joint_states',
                     JointState, self.joint_state_cb)
    rospy.Subscriber(robot_side_ns + 'dx200/dx200_r1_controller/joint_states',
                     JointState, self.r1_joint_state_cb)
    rospy.Subscriber(robot_side_ns + 'dx200/dx200_s1_controller/joint_states',
                     JointState, self.s1_joint_state_cb)

    rospy.Subscriber(robot_side_ns + 'feedback_states',
                     JointState, self.feedback_state_cb)
    rospy.Subscriber(robot_side_ns +'dx200/dx200_r1_controller/feedback_states',
                     JointState, self.r1_feedback_state_cb)
    rospy.Subscriber(robot_side_ns +'dx200/dx200_s1_controller/feedback_states',
                     JointState, self.s1_feedback_state_cb)
    rospy.Subscriber(robot_side_ns + 'dynamic_feedback_states',
                     JointState, self.feedback_state_cb)

    # Publish joint_states to rest of system
    self.joint_states_r1_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_r1_controller/joint_states', JointState, queue_size=1)
    self.joint_states_s1_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_s1_controller/joint_states', JointState, queue_size=1)

    self.angle_offsets_list = self.read_actuation_angle_offsets()
    rospy.loginfo("angle_offsets_list")
    rospy.loginfo(self.angle_offsets_list)

  def read_actuation_angle_offsets(self):
    rospack = rospkg.RosPack()
    ros_pkg_path = rospack.get_path('motoman_ma1400_support')
    path_to_yaml = ros_pkg_path + '/config/actuation_angle_offsets.yaml'
    try:
      angle_offsets_dict = yaml.load(open(path_to_yaml))['actuation_angle_offsets']
    except IOError:
      print("Error reading path_to_yaml: " + path_to_yaml)
      angle_offsets_dict = {}

    joints = ['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t', 'joint_x']
    angle_offsets_list = []
    for j in joints:
      try:
        angle_offsets_list.append(angle_offsets_dict[j])
      except KeyError:
        print(j + " not found in yaml file.")
    return angle_offsets_list

  def r1_joint_state_cb(self,msg):
    try:
      position = (msg.position[0] - self.angle_offsets_list[0],
                  msg.position[1] - self.angle_offsets_list[1],
                  msg.position[2] - self.angle_offsets_list[2],
                  msg.position[3] - self.angle_offsets_list[3],
                  msg.position[4] - self.angle_offsets_list[4],
                  msg.position[5] - self.angle_offsets_list[5])
      msg.position = position
      try:
        self.joint_states_r1_pub.publish(msg)
      except rospy.ROSException, e:
        rospy.logdebug(str(e))
    except AttributeError:
      rospy.logdebug("Class not initialized yet")

  def s1_joint_state_cb(self,msg):
    try:
      position = (msg.position[0] - self.angle_offsets_list[6], )
      msg.position = position
      try:
        self.joint_states_s1_pub.publish(msg)
      except rospy.ROSException, e:
        rospy.logdebug(str(e))
    except AttributeError:
      rospy.logdebug("Class not initialized yet")

  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node('actuation_angle_offsets')
  aao = actuation_angle_offsets()
  aao.spin()

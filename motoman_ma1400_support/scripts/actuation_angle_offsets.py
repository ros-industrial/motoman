#!/usr/bin/python
import rospy
import rospkg

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from motoman_msgs.msg import DynamicJointTrajectoryFeedback
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
    rospy.Subscriber(robot_side_ns + 'dx200/dx200_r1_controller/joint_states',
                     JointState, self.r1_joint_states_cb)
    rospy.Subscriber(robot_side_ns + 'dx200/dx200_s1_controller/joint_states',
                     JointState, self.s1_joint_states_cb)

    rospy.Subscriber(robot_side_ns + 'feedback_states',
                     FollowJointTrajectoryFeedback, self.feedback_states_cb)
    rospy.Subscriber(robot_side_ns +'dx200/dx200_r1_controller/feedback_states',
                     FollowJointTrajectoryFeedback, self.r1_feedback_states_cb)
    rospy.Subscriber(robot_side_ns +'dx200/dx200_s1_controller/feedback_states',
                     FollowJointTrajectoryFeedback, self.s1_feedback_states_cb)
    rospy.Subscriber(robot_side_ns + 'dynamic_feedback_states',
                     DynamicJointTrajectoryFeedback,
                     self.dynamic_feedback_states_cb)

    # Publish joint_states to rest of system
    self.r1_joint_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_r1_controller/joint_states', JointState, queue_size=1)
    self.s1_joint_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_s1_controller/joint_states', JointState, queue_size=1)

    self.feedback_states_pub = rospy.Publisher(comp_side_ns + 'feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.r1_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_r1_controller/feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.s1_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_s1_controller/feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.dynamic_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dynamic_feedback_states', DynamicJointTrajectoryFeedback, queue_size=1)

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

  def r1_joint_states_cb(self,msg):
    try:
      position = (msg.position[0] - self.angle_offsets_list[0],
                  msg.position[1] - self.angle_offsets_list[1],
                  msg.position[2] - self.angle_offsets_list[2],
                  msg.position[3] - self.angle_offsets_list[3],
                  msg.position[4] - self.angle_offsets_list[4],
                  msg.position[5] - self.angle_offsets_list[5])
      msg.position = position
      self.r1_joint_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def s1_joint_states_cb(self,msg):
    try:
      position = (msg.position[0] - self.angle_offsets_list[6], )
      msg.position = position
      self.s1_joint_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def feedback_states_cb(self,msg):
    try:
      num_joints = len(msg.actual.positions)
      if num_joints == 6:
        positions = (msg.actual.positions[0] - self.angle_offsets_list[0],
                     msg.actual.positions[1] - self.angle_offsets_list[1],
                     msg.actual.positions[2] - self.angle_offsets_list[2],
                     msg.actual.positions[3] - self.angle_offsets_list[3],
                     msg.actual.positions[4] - self.angle_offsets_list[4],
                     msg.actual.positions[5] - self.angle_offsets_list[5])
        msg.actual.positions = positions
      elif num_joints == 1:
        positions = (msg.actual.positions[0] - self.angle_offsets_list[6], )
        msg.actual.positions = positions
      else:
        rospy.logdebug("Unexpected number of joints: " + str(num_joints))
      self.feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def r1_feedback_states_cb(self,msg):
    try:
      positions = (msg.actual.positions[0] - self.angle_offsets_list[0],
                   msg.actual.positions[1] - self.angle_offsets_list[1],
                   msg.actual.positions[2] - self.angle_offsets_list[2],
                   msg.actual.positions[3] - self.angle_offsets_list[3],
                   msg.actual.positions[4] - self.angle_offsets_list[4],
                   msg.actual.positions[5] - self.angle_offsets_list[5])
      msg.actual.positions = positions
      self.r1_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.loginfo(str(e))

  def s1_feedback_states_cb(self,msg):
    try:
      positions = (msg.actual.positions[0] - self.angle_offsets_list[6], )
      msg.actual.positions = positions
      self.s1_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError:
      rospy.logdebug("Class not initialized yet")

  def dynamic_feedback_states_cb(self,msg):
    try:
      for i, fb in enumerate(msg.joint_feedbacks):
        if fb.group_number == 0:
          positions = (fb.positions[0] - self.angle_offsets_list[0],
                       fb.positions[1] - self.angle_offsets_list[1],
                       fb.positions[2] - self.angle_offsets_list[2],
                       fb.positions[3] - self.angle_offsets_list[3],
                       fb.positions[4] - self.angle_offsets_list[4],
                       fb.positions[5] - self.angle_offsets_list[5])
          msg.joint_feedbacks[i].positions = positions
        elif fb.group_number == 1:
          positions = (fb.positions[0] - self.angle_offsets_list[6], )
          msg.joint_feedbacks[i].positions = positions
        else:
          rospy.logdebug("group number " + str(fb.group_number) + " not recognized.")
      self.dynamic_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))
    except Exception, e:
      rospy.loginfo(str(e))

  def spin(self):
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node('actuation_angle_offsets')
  aao = actuation_angle_offsets()
  aao.spin()

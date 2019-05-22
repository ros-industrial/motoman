#!/usr/bin/python
import rospy
import rospkg

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from motoman_msgs.msg import DynamicJointTrajectoryFeedback
from motoman_msgs.msg import DynamicJointTrajectory

import yaml

class actuation_angle_offsets(object):
  """Add offsets to joint readings and subtract offsets from trajectories."""
  def __init__(self):
    """Initialize all the topics and read the offsets."""
    rospy.logdebug("initializing actuation_angle_offsets")
    # Get namespaces from the parameter server
    # robot_side_ns are the values coming directly from the motoman driver
    robot_side_ns = rospy.get_param('~robot_side_ns','offset')
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

    rospy.Subscriber(comp_side_ns + 'joint_path_command',
                     DynamicJointTrajectory, self.joint_path_command_cb)
    rospy.Subscriber(comp_side_ns + 'dx200/dx200_r1_controller/joint_path_command',
                     DynamicJointTrajectory, self.r1_joint_path_command_cb)
    rospy.Subscriber(comp_side_ns + 'dx200/dx200_s1_controller/joint_path_command',
                     DynamicJointTrajectory, self.s1_joint_path_command_cb)

    # Publish joint_states to rest of system
    self.r1_joint_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_r1_controller/joint_states', JointState, queue_size=1)
    self.s1_joint_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_s1_controller/joint_states', JointState, queue_size=1)

    self.feedback_states_pub = rospy.Publisher(comp_side_ns + 'feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.r1_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_r1_controller/feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.s1_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dx200/dx200_s1_controller/feedback_states', FollowJointTrajectoryFeedback, queue_size=1)
    self.dynamic_feedback_states_pub = rospy.Publisher(comp_side_ns + 'dynamic_feedback_states', DynamicJointTrajectoryFeedback, queue_size=1)

    self.joint_path_command_pub = rospy.Publisher(robot_side_ns + 'joint_path_command', DynamicJointTrajectory, queue_size=1)
    self.r1_joint_path_command_pub = rospy.Publisher(robot_side_ns + 'dx200/dx200_r1_controller/joint_path_command', DynamicJointTrajectory, queue_size=1)
    self.s1_joint_path_command_pub = rospy.Publisher(robot_side_ns + 'dx200/dx200_s1_controller/joint_path_command', DynamicJointTrajectory, queue_size=1)

    self.angle_offsets_list = self.read_actuation_angle_offsets()
    rospy.loginfo("angle_offsets_list")
    rospy.loginfo(self.angle_offsets_list)

  def read_actuation_angle_offsets(self):
    """
    Reads the angle offsets from a yaml file.

    Reads the angle offsets for joints SLURBTX from the file
    glugun_config/ma1400_sim.config/acutation_angle_offsets.yaml. These should
    be set from a robot calibration method.

    Returns:
    list of doubles: Angle offset values from yaml file
    """
    rospack = rospkg.RosPack()
    ros_pkg_path = rospack.get_path('glugun_config')
    path_to_yaml = ros_pkg_path + '/ma1400_sim.config/actuation_angle_offsets.yaml'
    try:
      angle_offsets_dict = yaml.load(open(path_to_yaml))['actuation_angle_offsets']
    except IOError:
      print("Error reading path_to_yaml: " + path_to_yaml)
      angle_offsets_dict = {}

    joints = ['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t', 'joint_1', 'joint_2']
    angle_offsets_list = []
    for j in joints:
      try:
        angle_offsets_list.append(angle_offsets_dict[j])
      except KeyError:
        print(j + " not found in yaml file.")
    return angle_offsets_list

  def r1_joint_states_cb(self,msg):
    """
    Add angle offsets to r1 joint sensor values (SLURBT).

    Parameters:
    msg (JointState): Joint state message
    """
    try:
      position = (msg.position[0] + self.angle_offsets_list[0],
                  msg.position[1] + self.angle_offsets_list[1],
                  msg.position[2] + self.angle_offsets_list[2],
                  msg.position[3] + self.angle_offsets_list[3],
                  msg.position[4] + self.angle_offsets_list[4],
                  msg.position[5] + self.angle_offsets_list[5])
      msg.position = position
      self.r1_joint_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def s1_joint_states_cb(self,msg):
    """
    Add angle offsets to s1 joint sensor values (X).

    Parameters:
    msg (JointState): Joint state message
    """
    try:
      position = (msg.position[0] + self.angle_offsets_list[6], )
      msg.position = position
      self.s1_joint_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def feedback_states_cb(self,msg):
    """
    Add angle offsets to joint sensor values (SLURBTX).

    Parameters:
    msg (FollowJointTrajectoryFeedback): Feedback state message
    """
    try:
      num_joints = len(msg.actual.positions)
      if num_joints == 6:
        positions = (msg.actual.positions[0] + self.angle_offsets_list[0],
                     msg.actual.positions[1] + self.angle_offsets_list[1],
                     msg.actual.positions[2] + self.angle_offsets_list[2],
                     msg.actual.positions[3] + self.angle_offsets_list[3],
                     msg.actual.positions[4] + self.angle_offsets_list[4],
                     msg.actual.positions[5] + self.angle_offsets_list[5])
        msg.actual.positions = positions
      elif num_joints == 1:
        positions = (msg.actual.positions[0] + self.angle_offsets_list[6], )
        msg.actual.positions = positions
      else:
        rospy.logdebug("Unexpected number of joints: " + str(num_joints))
      self.feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def r1_feedback_states_cb(self,msg):
    """
    Add angle offsets to r1 joint sensor values (SLURBT).

    Parameters:
    msg (FollowJointTrajectoryFeedback): Feedback state message
    """
    try:
      positions = (msg.actual.positions[0] + self.angle_offsets_list[0],
                   msg.actual.positions[1] + self.angle_offsets_list[1],
                   msg.actual.positions[2] + self.angle_offsets_list[2],
                   msg.actual.positions[3] + self.angle_offsets_list[3],
                   msg.actual.positions[4] + self.angle_offsets_list[4],
                   msg.actual.positions[5] + self.angle_offsets_list[5])
      msg.actual.positions = positions
      self.r1_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def s1_feedback_states_cb(self,msg):
    """
    Add angle offsets to s1 joint sensor values (X).

    Parameters:
    msg (FollowJointTrajectoryFeedback): Feedback state message
    """
    try:
      positions = (msg.actual.positions[0] + self.angle_offsets_list[6], )
      msg.actual.positions = positions
      self.s1_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError:
      rospy.logdebug("Class not initialized yet")

  def dynamic_feedback_states_cb(self,msg):
    """
    Add angle offsets to joint sensor values (SLURBTX).

    Parameters:
    msg (DynamicJointTrajectoryFeedback): Dynamic feedback state message
    """
    try:
      for i, fb in enumerate(msg.joint_feedbacks):
        if fb.group_number == 0:
          positions = (fb.positions[0] + self.angle_offsets_list[0],
                       fb.positions[1] + self.angle_offsets_list[1],
                       fb.positions[2] + self.angle_offsets_list[2],
                       fb.positions[3] + self.angle_offsets_list[3],
                       fb.positions[4] + self.angle_offsets_list[4],
                       fb.positions[5] + self.angle_offsets_list[5])
          msg.joint_feedbacks[i].positions = positions
        elif fb.group_number == 1:
          positions = (fb.positions[0] + self.angle_offsets_list[6], )
          msg.joint_feedbacks[i].positions = positions
        else:
          rospy.logdebug("group number " + str(fb.group_number) + " not recognized.")
      self.dynamic_feedback_states_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))
    except Exception, e:
      rospy.logdebug(str(e))

  def joint_path_command_cb(self,msg):
    """
    Subtract angle offsets from joint path command (SLURBTX).

    Parameters:
    msg (DynamicJointTrajectory): Dynamic joint path command message
    """
    try:
      for i, pt in enumerate(msg.points):
        for j, gp in enumerate(pt.groups):
          if gp.group_number == 0:
            positions = (gp.positions[0] - self.angle_offsets_list[0],
                         gp.positions[1] - self.angle_offsets_list[1],
                         gp.positions[2] - self.angle_offsets_list[2],
                         gp.positions[3] - self.angle_offsets_list[3],
                         gp.positions[4] - self.angle_offsets_list[4],
                         gp.positions[5] - self.angle_offsets_list[5])
            msg.points[i].groups[j].positions = positions
          elif gp.group_number == 1:
            positions = (gp.positions[0] - self.angle_offsets_list[6], )
            msg.points[i].groups[j].positions = positions
          else:
            rospy.logdebug("group number " + str(fb.group_number) + " not recognized.")
      self.joint_path_command_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def r1_joint_path_command_cb(self,msg):
    """
    Subtract angle offsets from r1 joint path command (SLURBT).

    Parameters:
    msg (DynamicJointTrajectory): Dynamic joint path command message
    """
    try:
      ordered_angle_offsets_list = self.list_in_joint_order(msg.joint_names)
      for i, pt in enumerate(msg.points):
        for j, gp in enumerate(pt.groups):
          if gp.group_number == 0:
            positions = (gp.positions[0] - ordered_angle_offsets_list[0],
                         gp.positions[1] - ordered_angle_offsets_list[1],
                         gp.positions[2] - ordered_angle_offsets_list[2],
                         gp.positions[3] - ordered_angle_offsets_list[3],
                         gp.positions[4] - ordered_angle_offsets_list[4],
                         gp.positions[5] - ordered_angle_offsets_list[5])
            msg.points[i].groups[j].positions = positions
          elif gp.group_number == 1:
            positions = (gp.positions[0] - self.angle_offsets_list[6], )
            msg.points[i].groups[j].positions = positions
          else:
            rospy.logdebug("group number " + str(fb.group_number) + " not recognized.")
      self.r1_joint_path_command_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def s1_joint_path_command_cb(self,msg):
    """
    Subtract angle offsets from s1 joint path command (X).

    Parameters:
    msg (DynamicJointTrajectory): Dynamic joint path command message
    """
    try:
      ordered_angle_offsets_list = self.list_in_joint_order(msg.joint_names)
      for i, pt in enumerate(msg.points):
        for j, gp in enumerate(pt.groups):
          if gp.group_number == 0:
            positions = (gp.positions[0] - ordered_angle_offsets_list[0],
                         gp.positions[1] - ordered_angle_offsets_list[1],
                         gp.positions[2] - ordered_angle_offsets_list[2],
                         gp.positions[3] - ordered_angle_offsets_list[3],
                         gp.positions[4] - ordered_angle_offsets_list[4],
                         gp.positions[5] - ordered_angle_offsets_list[5])
            msg.points[i].groups[j].positions = positions
          elif gp.group_number == 1:
            positions = (gp.positions[0] - self.angle_offsets_list[6], )
            msg.points[i].groups[j].positions = positions
          else:
            rospy.logdebug("group number " + str(fb.group_number) + " not recognized.")
      self.s1_joint_path_command_pub.publish(msg)
    except rospy.ROSException, e:
      rospy.logdebug(str(e))
    except AttributeError, e:
      rospy.logdebug(str(e))

  def list_in_joint_order(self, joint_order):
    """
    Given a list of joint names, return the angle offsets in that order

    Parameters:
    joint_order (list of strings): The names of the joints in a specific order

    Returns:
    list of doubles: The angle offsets in the order specified by joint_order
    """
    list_order = ['joint_s', 'joint_l', 'joint_u', 'joint_r', 'joint_b', 'joint_t', 'joint_1', 'joint_2']
    ordered_list = []
    for jt in joint_order:
      idx = list_order.index(jt)
      ordered_list.append(self.angle_offsets_list[idx])
    return ordered_list

  def spin(self):
    """Spins up the node to listen for incoming messages."""
    rospy.spin()


if __name__ == "__main__":
  rospy.init_node('actuation_angle_offsets')
  aao = actuation_angle_offsets()
  aao.spin()

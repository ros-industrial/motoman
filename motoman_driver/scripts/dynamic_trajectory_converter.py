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
into a unified higher dof message and publish. On the flip side, take a
higher DOF joint_path_command of type <JointTrajectory> and break it down
into a <DynamicJointTrajectory> joint message

MotoMan by default sets up a FollowJointTrajectory action server for each
control group of the type <DynamicJointTrajectory>, while MoveIt! and just
about everything else works in just a standard <JointTrajectory>. As part
of the MoveIt! config, a top level controller is defined that contains all
joints in the system (see config/controllers.yaml in of the moveit_config
packages). This top level control group operates in the namespace defined
by the name of the controller and interacts with a joint_trajectory_action
node of the standard <JointTrajectory> type. Launch files should define a
'robot_ns' parameters that coincides with the name of the combined MoveIt!
controller. This node acts as a middleman between the individual
joint_trajectory_action nodes for each motoman control group and the top
level combined controller. It listens to the joint_states and feedback_states
of each control group and aggregrates them into a single message on each topic
for the combined control group. Likewise, it listens on the
<robot_ns>/joint_path_command topic for standard <JointTrajectory> messages
and converts them to a motoman compatible <DynamicJointTrajectory> message
that then gets published to the robot.

The motoman driver package supports up to 4 separate control groups (at least
on the DX200), so this node operates on that assumption and that the group
number ranges from 0-3
'''


class dynamic_trajectory_converter(object):
    def __init__(self):
        # this is the namespace the robot operates in within MoveIt!, this param
        # is required
        robot_ns = rospy.get_param('robot_ns', '')
        if robot_ns:  # If the namspace isn't empty, add a '/'
            robot_ns = robot_ns + "/"
        else:
            rospy.FATAL('param \'robot_ns\' is not defined!')

        # motoman sets up a separate FollowJointTrajectory action server for
        # each group in '/topic_list'
        self.motoman_topic_list = rospy.get_param('/topic_list')
        self.num_control_groups = len(self.motoman_topic_list)

        # Locks to prevent pub_joint_states and pub_feedback_states from
        # running multiple instances at the same time.
        self.joint_states_lock = threading.Lock()
        self.feedback_states_lock = threading.Lock()
        self.robot_status_lock = threading.Lock()

        # the DX200 can support up to four control groups (see motoman_driver/MotoPlus/Controller.h),
        # set up a group in the dict for each
        group_joint_state_cb_dict = {
            0: self.group0_joint_state_cb,
            1: self.group1_joint_state_cb,
            2: self.group2_joint_state_cb,
            3: self.group3_joint_state_cb
        }

        group_feedback_state_cb_dict = {
            0: self.group0_feedback_state_cb,
            1: self.group1_feedback_state_cb,
            2: self.group2_feedback_state_cb,
            3: self.group3_feedback_state_cb
        }

        group_robot_status_cb_dict = {
            0: self.group0_robot_status_cb,
            1: self.group1_robot_status_cb,
            2: self.group2_robot_status_cb,
            3: self.group3_robot_status_cb
        }

        # Subscribe to the robot status
        rospy.Subscriber('robot_status', RobotStatus, self.robot_status_cb)

        # Publish robot staus and joint states on a new topic scoped
        # to /<robot_ns> namespace
        self.robot_joint_states_pub = rospy.Publisher(
            robot_ns + 'joint_states', JointState, queue_size=1)

        self.robot_status_pub = rospy.Publisher(
            robot_ns + 'robot_status', RobotStatus, queue_size=1)

        # Publish joint_states to rest of system
        self.feedback_states_pub = rospy.Publisher(robot_ns + 'feedback_states',
                                                   FollowJointTrajectoryFeedback,
                                                   queue_size=1)

        # Maintain a memory of the most recent joint states and feedback states
        self.group_joint_states = {
            0: JointState(),
            1: JointState(),
            2: JointState(),
            3: JointState()
        }

        self.group_feedback_states = {
            0: FollowJointTrajectoryFeedback(),
            1: FollowJointTrajectoryFeedback(),
            2: FollowJointTrajectoryFeedback(),
            3: FollowJointTrajectoryFeedback()
        }

        self.group_robot_status = {
            0: RobotStatus(),
            1: RobotStatus(),
            2: RobotStatus(),
            3: RobotStatus()
        }

        # Subscribe to joint_path_command from system
        rospy.Subscriber(robot_ns + 'joint_path_command', JointTrajectory,
                         self.joint_path_command_cb)
        # Publish joint_path_command to robot/simulator
        self.joint_path_command_pub = rospy.Publisher('joint_path_command',
                                                      DynamicJointTrajectory,
                                                      queue_size=1)

        # Publish joint states on top level /joint_states topic as well, needed
        # within moveit core to get current robot state
        self.joint_states_pub = rospy.Publisher(
            'joint_states', JointState, queue_size=1)

        # set up subscribers to joint/feedback_states based on group #
        # do this last to prevent incoming messages from triggering 
        # callbacks and throwing intermittent errors based on timing
        for topic in self.motoman_topic_list:
            rospy.Subscriber('/' + topic["ns"] + '/' + topic["name"] +
                             '/joint_states',
                             JointState,
                             group_joint_state_cb_dict[topic["group"]])
            rospy.Subscriber('/' + topic["ns"] + '/' + topic["name"] +
                             '/feedback_states',
                             FollowJointTrajectoryFeedback,
                             group_feedback_state_cb_dict[topic["group"]])
            rospy.Subscriber('/' + topic["ns"] + '/' + topic["name"] +
                             '/robot_status',
                             RobotStatus,
                             group_robot_status_cb_dict[topic["group"]])

    # Store the joint state in memory and publish if they are close, temporally
    def group0_joint_state_cb(self, msg):
        self.joint_states_lock.acquire()
        self.group_joint_states[0] = msg
        self.pub_joint_states()
        self.joint_states_lock.release()

    def group1_joint_state_cb(self, msg):
        self.joint_states_lock.acquire()
        self.group_joint_states[1] = msg
        self.pub_joint_states()
        self.joint_states_lock.release()

    def group2_joint_state_cb(self, msg):
        self.joint_states_lock.acquire()
        self.group_joint_states[2] = msg
        self.pub_joint_states()
        self.joint_states_lock.release()

    def group3_joint_state_cb(self, msg):
        self.joint_states_lock.acquire()
        self.group_joint_states[3] = msg
        self.pub_joint_states()
        self.joint_states_lock.release()

    # aggregate the joint_state messages from the individual Motoman topics
    # and publish them as a combined single message to MoveIt! and whatever
    # else in they system within the <robot_ns> namespace
    def pub_joint_states(self):
        # Check to see if two joint states share the same sequence number
        # Should protect against running before all messages have been received
        if self.check_callback_sequences_match(self.group_joint_states):
            # Combine into a higher dof message
            joint_states = JointState()

            joint_states.header.stamp = self.get_min_header_timestamp(
                self.group_joint_states)

            joint_names = self.get_all_joint_names()
            positions = []
            velocities = []

            # for group in self.group_joint_states:
            for i in range(0, self.num_control_groups):
                positions = positions + \
                    list(self.group_joint_states[i].position)
                velocities = velocities + \
                    list(self.group_joint_states[i].velocity)

            joint_states.name = joint_names
            joint_states.position = positions
            joint_states.velocity = velocities
            # And publish to a new topic
            self.joint_states_pub.publish(joint_states)
            self.robot_joint_states_pub.publish(joint_states)

    # Store the feedback state in memory and publish if they are close
    def group0_feedback_state_cb(self, msg):
        self.feedback_states_lock.acquire()
        self.group_feedback_states[0] = msg
        self.pub_feedback_states()
        self.feedback_states_lock.release()

    def group1_feedback_state_cb(self, msg):
        self.feedback_states_lock.acquire()
        self.group_feedback_states[1] = msg
        self.pub_feedback_states()
        self.feedback_states_lock.release()

    def group2_feedback_state_cb(self, msg):
        self.feedback_states_lock.acquire()
        self.group_feedback_states[2] = msg
        self.pub_feedback_states()
        self.feedback_states_lock.release()

    def group3_feedback_state_cb(self, msg):
        self.feedback_states_lock.acquire()
        self.group_feedback_states[3] = msg
        self.pub_feedback_states()
        self.feedback_states_lock.release()

    # aggregate the feedback_state messages from the individual Motoman topics
    # and publish them as a combined single message to MoveIt! and whatever
    # else in they system within the <robot_ns> namespace
    def pub_feedback_states(self):
        # Check to see if two feedback states share the same sequence number
        # Should protect against running before both messages have been
        # received
        if self.check_callback_sequences_match(self.group_feedback_states):
            # Combine into a 7dof message
            feedback_states = FollowJointTrajectoryFeedback()

            feedback_states.header.stamp = self.get_min_header_timestamp(
                self.group_feedback_states)

            joint_names = self.get_all_joint_names()
            positions = []
            velocities = []

            for i in range(0, self.num_control_groups):
                positions = positions + list(self.group_feedback_states[i]
                                             .actual.positions)
                velocities = velocities + list(self.group_feedback_states[i]
                                               .actual.velocities)

            feedback_states.joint_names = joint_names
            feedback_states.actual.positions = positions
            feedback_states.actual.velocities = velocities
            # And publish to a new topic
            self.feedback_states_pub.publish(feedback_states)

    # Store the feedback state in memory and publish if they are close
    def group0_robot_status_cb(self, msg):
        self.robot_status_lock.acquire()
        self.group_robot_status[0] = msg
        self.pub_robot_status()
        self.robot_status_lock.release()

    def group1_robot_status_cb(self, msg):
        self.robot_status_lock.acquire()
        self.group_robot_status[1] = msg
        self.pub_robot_status()
        self.robot_status_lock.release()

    def group2_robot_status_cb(self, msg):
        self.robot_status_lock.acquire()
        self.group_robot_status[2] = msg
        self.pub_robot_status()
        self.robot_status_lock.release()

    def group3_robot_status_cb(self, msg):
        self.robot_status_lock.acquire()
        self.group_robot_status[3] = msg
        self.pub_robot_status()
        self.robot_status_lock.release()

    # aggregate the robot_status messages from the individual Motoman topics
    # and publish them as a combined single message to MoveIt! and whatever
    # else in they system within the <robot_ns> namespace
    def pub_robot_status(self):
        # Check to see if two feedback states share the same sequence number
        # Should protect against running before both messages have been
        # received
        if self.check_callback_sequences_match(self.group_robot_status):
            robot_status = RobotStatus()

            robot_status.header.stamp = self.get_min_header_timestamp(
                self.group_robot_status)

            robot_status.e_stopped.val = 1 if any(x.e_stopped.val == 1 for x in self.group_robot_status.values()) else 0
            robot_status.drives_powered.val = 1 if any(
                x.drives_powered.val == 1 for x in self.group_robot_status.values()) else 0
            robot_status.motion_possible.val = 1 if any(
                x.motion_possible.val == 1 for x in self.group_robot_status.values()) else 0
            robot_status.in_motion.val = 1 if any(x.in_motion.val == 1 for x in self.group_robot_status.values()) else 0
            robot_status.in_error.val = 1 if any(x.in_error.val == 1 for x in self.group_robot_status.values()) else 0

            # And publish to a new topic
            self.robot_status_pub.publish(robot_status)

    def joint_path_command_cb(self, msg):
        # Divide the higher dof message into separate groups
        points = ()
        for i, dyn_point in enumerate(msg.points):

            point = DynamicJointPoint()
            groups = []

            # loop through each control group to form a DynamicJointsGroup.msg
            for ctrl_group in self.motoman_topic_list:
                group = DynamicJointsGroup()
                num_joints_in_group = len(ctrl_group['joints'])

                positions = []
                velocities = []
                accelerations = []

                # the combined message comes out in alphabetical order, so we
                # have to loop through each joint in each control group and
                # grab the appropriate position, velocity, and acceleration
                # values by index
                for joint in ctrl_group['joints']:
                    joint_ind = msg.joint_names.index(joint)

                    positions.append(dyn_point.positions[joint_ind])

                    # vectors will either be empty or should have the same number
                    # of elements as the positions vector
                    if dyn_point.velocities:
                        velocities.append(dyn_point.velocities[joint_ind])
                    if dyn_point.accelerations:
                        accelerations.append(dyn_point.accelerations[joint_ind])

                group.group_number = ctrl_group['group']
                group.num_joints = num_joints_in_group
                group.valid_fields = 0
                group.positions = positions
                group.velocities = velocities
                group.accelerations = accelerations
                group.time_from_start = dyn_point.time_from_start

                groups.append(group)

            # aggregrate groups into a DynamicJointPoint
            point.num_groups = self.num_control_groups
            point.groups = groups

            # stuff it onto the end of the trajectory
            points = points + (point,)

        # rospy.loginfo(points)
        dyn_traj = DynamicJointTrajectory()
        dyn_traj.header = msg.header
        dyn_traj.joint_names = self.get_all_joint_names()
        dyn_traj.points = points
        # and publish
        self.joint_path_command_pub.publish(dyn_traj)

    # this subscribes to the top-level /robot_status topic published to by
    # the actual motoman robot and publishes it back out to MoveIt! and the
    # rest of the system on the <robot_ns> namespace
    def robot_status_cb(self, msg):
        self.robot_status_pub.publish(msg)

    # check that header sequences of each joint_state or feedback_state callback
    #  match up before re-publishing everything back out on aggregated topics
    def check_callback_sequences_match(self, state_dict):
        all_seq_match = True

        for i in range(0, self.num_control_groups - 1):
            if state_dict[i].header.seq != state_dict[i + 1].header.seq:
                all_seq_match = False
                break

        return all_seq_match

    # get the minimum timestamp between the control groups to send back out on
    # the aggregrated messages
    def get_min_header_timestamp(self, ctrl_groups_dict):
        min_stamp = ctrl_groups_dict[0].header.stamp

        for i in range(1, self.num_control_groups):
            if ctrl_groups_dict[i].header.stamp < min_stamp:
                min_stamp = ctrl_groups_dict[i].header.stamp

        return min_stamp

    # combine joint names from all groups into one list
    def get_all_joint_names(self):
        combined_joint_names = []

        for group in self.motoman_topic_list:
            combined_joint_names = combined_joint_names + group['joints']

        return combined_joint_names

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('dynamic_trajectory_converter')
    dtj = dynamic_trajectory_converter()
    dtj.spin()

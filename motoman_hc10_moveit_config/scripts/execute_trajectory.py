#! /usr/bin/env python
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory',anonymous=True)

#Misc variables
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

#Group states taken from the srdf file
group_state = group.get_named_targets()

#Planning and executing with set_joint_value_target
print ("Number of group states in srdf file: %i \n" % len(group_state))
for n in range(0,len(group_state)): #Home configuration (i.e 0 position) is a singularity 
    if not rospy.is_shutdown():
        print ("group state %i: %s" %(n,group_state[n]))
        print ("Joint Values %s" %group.get_named_target_values(group_state[n]))
        group.set_joint_value_target(group.get_named_target_values(group_state[n]))
        print ("New target has been set")
        plan2 = group.plan()
        rospy.sleep(1)
        # If you want to move the group to the specified targets uncomment the lines below
        print ("Plannig done, now executing \n") 
        group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
        group.stop()
        rospy.sleep(1)

moveit_commander.roscpp_shutdown()








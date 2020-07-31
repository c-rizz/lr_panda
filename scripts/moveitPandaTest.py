#!/usr/bin/env python2

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

planning_frame = move_group.get_planning_frame()
print("============ Planning frame: "+planning_frame)
eef_link = move_group.get_end_effector_link()
print("============ End effector link: "+eef_link)
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())
print("============ Printing robot state")
print(robot.get_current_state())
print("")
## END_SUB_TUTORIAL

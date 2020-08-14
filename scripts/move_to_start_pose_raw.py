#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from controller_manager_msgs.srv import ListControllers


def buildServiceProxy(serviceName, msgType):
    rospy.wait_for_service(serviceName)
    return rospy.ServiceProxy(serviceName, msgType)

def waitForControllersStart(controllerNames, listControllers_service):
    allStarted = False
    while not allStarted:
        res = listControllers_service()
        loadedControllers = res.controller
        allStarted = True
        # Check one by one that each neededController is loaded and started, if even one is not, sleep and retry
        for neededControllerName in controllerNames:
            neededController = None
            for c in loadedControllers:
                if c.name == neededControllerName:
                    neededController = c
            if neededController is None:
                rospy.logwarn("Controller "+neededControllerName+" not loaded, will wait...")
                allStarted = False
                break
            if neededController.state != "running":
                rospy.logwarn("Controller "+neededControllerName+" not started, will wait...")
                allStarted = False
                break
        if not allStarted:
            rospy.sleep(1)


rospy.init_node('move_to_start_pose', anonymous=True, log_level=rospy.INFO)

controllerActionName = '/panda_arm_effort_trajectory_controller/follow_joint_trajectory'
controllerClient = actionlib.SimpleActionClient(controllerActionName, FollowJointTrajectoryAction)
rospy.logwarn("Waiting for "+str(controllerActionName)+"...")
controllerClient.wait_for_server(rospy.Duration(30))

listControllers_service = buildServiceProxy("/controller_manager/list_controllers", ListControllers)
waitForControllersStart(["panda_arm_effort_trajectory_controller"], listControllers_service)


goal = FollowJointTrajectoryGoal()
goal.trajectory.joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
point = JointTrajectoryPoint()
point.positions = [0, 0, 0, -1, 0, 1, 0]
point.effort = [0, 0, 0, 0, 0, 0, 0]
point.time_from_start = rospy.Duration(10)
goal.trajectory.points = [point]

controllerClient.send_goal(goal)
rospy.logwarn("Waiting for action completion...")
controllerClient.wait_for_result(point.time_from_start+rospy.Duration(5))
r = controllerClient.get_result()
rospy.logwarn("Action finished or timed out")

if r.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
    rospy.loginfo("Moved successfully to start pose")
else:
    rospy.logerr("Failed to move to start pose. Result is: "+str(r))
    exit(1)

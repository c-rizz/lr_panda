#!/usr/bin/env python3

import rospy
import actionlib
import argparse
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from controller_manager_msgs.srv import ListControllers
from typing import List

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
            rospy.logwarn("Waiting 1s")
            rospy.sleep(1)
            rospy.logwarn("Waited 1s")


def move(joint_names : List[str], positions : List[float]):
    controllerActionName = controllerNamespace+"/follow_joint_trajectory"
    controllerClient = actionlib.SimpleActionClient(controllerActionName, FollowJointTrajectoryAction)
    connected = False
    while not connected:
        rospy.logwarn("Waiting for "+str(controllerActionName)+"...")
        rospy.sleep(1.0)
        rospy.logwarn("Waited 1s")
        connected = controllerClient.wait_for_server(rospy.Duration(5.0))
    rospy.loginfo("Connected")

    listControllers_service = buildServiceProxy("controller_manager/list_controllers", ListControllers)
    waitForControllersStart([controllerNamespace], listControllers_service)
    rospy.loginfo("Controller ready")


    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joint_names

    #print("args['joint_pose'] = "+str(args["joint_pose"]))

    point = JointTrajectoryPoint()
    point.positions = positions
    point.effort = [0, 0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration(10)
    goal.trajectory.points = [point]

    rospy.loginfo("Sending goal")
    controllerClient.send_goal(goal)
    rospy.logwarn("Waiting for action completion...")
    r = controllerClient.wait_for_result(point.time_from_start+rospy.Duration(5))
    if r:
        if controllerClient.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.loginfo("Moved successfully to start pose")
    else:
        rospy.logerr("Failed to move to start pose. Result is: "+str(controllerClient.get_result()))
        return -1
    return 0

if __name__ == "__main__":
    rospy.init_node('move_to_joint_pose_raw', anonymous=True, log_level=rospy.INFO)
    ap = argparse.ArgumentParser()
    ap.add_argument("--traj_controller_name", required=False, default="panda_arm_effort_trajectory_controller", type=str, help="Topic namespace name for the trajectory contorller to use")
    ap.add_argument("--robot_name", required=False, default="panda", type=str, help="Topic namespace name for the trajectory contorller to use")
    ap.add_argument("--joint_prefix", required=False, default="joint", type=str, help="Naming of the joints")
    ap.add_argument('--joint_pose', nargs='+', type=float)
    args = vars(ap.parse_known_args()[0])

    controllerNamespace = args["traj_controller_name"]
    assert len(args["joint_pose"])==7, "joint_pose should be composed of 7 floats, it currently is "+str(args["joint_pose"])

    joint_names = [ args["robot_name"]+'_'+args["joint_prefix"]+'1',
                    args["robot_name"]+'_'+args["joint_prefix"]+'2',
                    args["robot_name"]+'_'+args["joint_prefix"]+'3',
                    args["robot_name"]+'_'+args["joint_prefix"]+'4',
                    args["robot_name"]+'_'+args["joint_prefix"]+'5',
                    args["robot_name"]+'_'+args["joint_prefix"]+'6',
                    args["robot_name"]+'_'+args["joint_prefix"]+'7']
    positions = args["joint_pose"]

    r = move(joint_names, positions)
    if r<0:
        exit(1)

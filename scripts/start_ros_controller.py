#!/usr/bin/env python


import rospy
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest


rospy.init_node('startup_setup_panda_sim', anonymous=True)
controllerNames = rospy.get_param("~controllers")
rospy.logwarn(str(controllerNames))
controllerNames = controllerNames.strip("[ ]")
rospy.logwarn(str(controllerNames))
controllerNames = controllerNames.split(",")
rospy.logwarn(str(controllerNames))
controllerNames = [ cn.strip(' ') for cn in controllerNames]
rospy.logwarn(str(controllerNames))


serviceName = "/controller_manager/switch_controller"
rospy.wait_for_service(serviceName)

switchController_service   = rospy.ServiceProxy(serviceName, SwitchController)

request = SwitchControllerRequest()
request.start_controllers = controllerNames
request.stop_controllers = [""]
request.strictness = 0
request.start_asap = False
request.timeout = 0.0
rospy.logwarn(str(request))
response = switchController_service(request)
rospy.logwarn(""+serviceName+" responded: '"+str(response)+"'")

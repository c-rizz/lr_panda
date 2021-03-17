#!/usr/bin/env python3


import rospy
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchControllerRequest

def buildServiceProxy(serviceName, msgType):
    rospy.wait_for_service(serviceName)
    return rospy.ServiceProxy(serviceName, msgType)


def waitForControllersLoad(controllerNames, listControllers_service):
    allLoaded = False
    while not allLoaded:
        res = listControllers_service()
        loadedControllerNames = [c.name for c in res.controller]
        allLoaded = True
        for neededController in controllerNames:
            if neededController not in loadedControllerNames:
                rospy.logwarn("Controller "+neededController+" not available, will wait...")
                allLoaded = False
                break
        if not allLoaded:
            rospy.sleep(1)


rospy.init_node('startup_setup_panda_sim', anonymous=True)
controllerNames = str(rospy.get_param("~controllers"))
controllerNames = controllerNames.strip("[ ]")
controllerNames = controllerNames.split(", ")
controllerNames = [ cn.strip("' ") for cn in controllerNames]



switchController_service = buildServiceProxy("/controller_manager/switch_controller", SwitchController)
listControllers_service = buildServiceProxy("/controller_manager/list_controllers", ListControllers)

waitForControllersLoad(controllerNames, listControllers_service)
rospy.loginfo("All controllers available")

request = SwitchControllerRequest()
request.start_controllers = controllerNames
request.stop_controllers = []
request.strictness = SwitchControllerRequest.STRICT
request.start_asap = False
request.timeout = 0.0
rospy.logdebug(str(request))
response = switchController_service(request)
if response.ok:
    rospy.loginfo("All Controllers started successfully")
else:
    rospy.logerror("Failed to start controllers")

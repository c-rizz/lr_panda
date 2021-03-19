#!/usr/bin/env python3

import rospy

rospy.init_node('print_erorr', anonymous=True)
msg = str(rospy.get_param("~msg"))
rospy.logerr(msg)
#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from ur_msgs.srv import *

def setio_client():
    
    rospy.wait_for_service('ur_hardware_interface/set_io')
    rospy.init_node('open_gripper')
    try:
        setio = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)
        resp1 = setio(fun=1, pin=17, state=24)
        rospy.sleep(1)
        resp1 = setio(fun=1, pin= 17, state=0)
        rospy.sleep(1)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
        setio_client()

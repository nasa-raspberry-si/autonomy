#!/usr/bin/env python

import rospy
from fault_management.fault_management_service import FaultManagementService

def fault_management():
    rospy.init_node('fault_management_node')
    rospy.loginfo("[Fault Management Node] initialized")

    fault_management_service = FaultManagementService()

    rospy.loginfo("[Fault Management Node] Ready to conduct fault management")
    rospy.spin()


if __name__ == "__main__":
    fault_management()

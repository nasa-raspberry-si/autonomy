#!/usr/bin/env python

from __future__ import print_function

import os
import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import ArmFaultConfig, ArmFaultConfigResponse

import dynamic_reconfigure.client

class FaultManagement:
    def __init__(self):
        client = dynamic_reconfigure.client.Client('/faults')
        arm_faults_vars_dict = {
            1: "dist_pitch_joint_locked_failure",
            2: "hand_yaw_joint_locked_failure",
            3: "prox_pitch_joint_locked_failure",
            4: "scoop_yaw_joint_locked_failure",
            5: "shou_pitch_joint_locked_failure",
            6: "shou_yaw_joint_locked_failure"
        }
        fm_service = rospy.Service(
            '/arm_fault_management',
            ArmFaultConfig,
            self.callback_arm_fault_management)

    def set_parameter_arm_fault_config(self, arm_fault_var_ids, values):
        params = {}
        for idx in arm_fault_var_ids:
            params[self.arm_faults_vars_dict[idx]] = values[idx]
        return params

    def callback_arm_fault_management(self, req):
    {
        success = True
        params = self.set_parameter_arm_fault_config(req.arm_fault_var_ids, req.values)
        if req.action in ["Inject", "Clear"]:
            loginfo("[Fault Management For Evaluation] " + req.action)
            config = self.client.update_configuration(params) 
        else:
            loginfo("[Fault Management For Evaluation] unknown action: " + req.action)
            success = False

        return ArmFaultConfigResponse(success)
    }



def fault_management():
    rospy.init_node('fault_management_node')

    fault_manager = FaultManagement()

    rospy.loginfo("Ready to conduct fault management")
    rospy.spin()


if __name__ == "__main__":
    fault_management()

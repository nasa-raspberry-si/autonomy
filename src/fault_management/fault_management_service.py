import os
import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import ArmFaultConfig, ArmFaultConfigResponse

import dynamic_reconfigure.client

class FaultManagementService:
    def __init__(self):
        self.faults_config_client = dynamic_reconfigure.client.Client('/faults')

        self.arm_faults_vars_dict = {
            1: "dist_pitch_joint_locked_failure",
            2: "hand_yaw_joint_locked_failure",
            3: "prox_pitch_joint_locked_failure",
            4: "scoop_yaw_joint_locked_failure",
            5: "shou_pitch_joint_locked_failure",
            6: "shou_yaw_joint_locked_failure"
        }

        self.fm_service = rospy.Service(
            '/arm_fault_management',
            ArmFaultConfig,
            self.callback_arm_fault_management)
        rospy.loginfo("[Fault Management Node] service '/arm_fault_management' is ready")


    def set_parameter_arm_fault_config(self, arm_fault_var_ids, values):
        params = {}
        for idx in arm_fault_var_ids:
            params[self.arm_faults_vars_dict[idx]] = values[idx]
        return params

    def callback_arm_fault_management(self, req):
        success = True
        params = self.set_parameter_arm_fault_config(req.arm_fault_var_ids, req.values)
        if req.action in ["Inject", "Clear"]:
            loginfo("[Fault Management For Evaluation] " + req.action)
            config = self.faults_config_client.update_configuration(params) 
        else:
            loginfo("[Fault Management For Evaluation] unknown action: " + req.action)
            success = False

        return ArmFaultConfigResponse(success)

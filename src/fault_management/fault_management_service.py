import os
import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import ArmFaultConfig, ArmFaultConfigResponse

import dynamic_reconfigure.client

class FaultManagementService:
    def __init__(self):
        self.faults_config_client = dynamic_reconfigure.client.Client('/faults')

        # ow_simulator Release 8
        # Fault Injection and Modeling:
        # https://github.com/nasa/ow_simulator/wiki/Fault-Injection-and-Modeling/2f5ede92df2704a68d7a1b0436e2dc81612a29c5
        self.arm_faults_vars_dict = {
                1: "dist_pitch_encoder_failure",
                2: "dist_pitch_effort_failure",
                3: "hand_yaw_encoder_failure",
                4: "hand_yaw_effort_failure",
                5: "prox_pitch_encoder_failure",
                6: "prox_pitch_effort_failure",
                7: "scoop_yaw_encoder_failure",
                8: "scoop_yaw_effort_failure",
                9: "shou_pitch_encoder_failure",
                10: "shou_pitch_effort_failure",
                11: "shou_yaw_encoder_failure",
                12: "shou_yaw_effort_failure"
        }

        #    ow_simulator Release 9
        #    1: "dist_pitch_joint_locked_failure",
        #    2: "hand_yaw_joint_locked_failure",
        #    3: "prox_pitch_joint_locked_failure",
        #    4: "scoop_yaw_joint_locked_failure",
        #    5: "shou_pitch_joint_locked_failure",
        #    6: "shou_yaw_joint_locked_failure"
        #}

        self.fm_service = rospy.Service(
            '/arm_fault_management',
            ArmFaultConfig,
            self.callback_arm_fault_management)
        rospy.loginfo("[Fault Management Node] service '/arm_fault_management' is ready")


    def set_parameter_arm_fault_config(self, arm_fault_var_ids, fault_values):
        params = {}

        for fault_id, fault_value in zip(arm_fault_var_ids, fault_values):
            params[fault_id] = fault_value
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

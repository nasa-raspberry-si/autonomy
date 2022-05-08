#!/usr/bin/env python

from __future__ import print_function

import os
import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import PlanTranslation, PlanTranslationResponse

from plan-translation.plexil_plan_translator import PlexilPlanTranslator

evaluation_root_dir = os.environ("EVALUATION_ROOT_DIR")


class PlanTranslation:
    def __init__(self):
        translation_service = rospy.Service(
                '/plan_translation',
                PlanTranslation,
                self.callback_plan_translation)

    def __translation(self, task_name, plan_name, high_level_plan):
        # Step 1
        # * Load runtime info file
        loginfo("[Plan Translation - Step 1] Loading the run-time information for plan translation.")
        task_dir = evaluation_root_dir + "/" + task_name
        current_plan_dir = task_dir + "/" + plan_name
        rt_info_fp = current_plan_dir + "/" + "rt_info.json"
        runtime_info = {}
        with open(rt_info_fp) as infile:
            runtime_info = json.load(infile)

        # Step 2
        # * Parse the high-level plan
        # * Call PLEXIL plan translation utility
        loginfo("[Plan Translation - Step 2] High-level plan parsing and PLEXIL plan writeup.")
        plexil_plan_translator = PlexilPlanTranslator()
        plexil_plan_dir = result_dir
        plexil_plan_translator.translate(
                task_name,
                runtime_info,
                plan_name,
                current_plan_dir,
                high_level_plan)
        

    def callback_plan_translation(self, req):
    {
        task_name = req.task_name
        plan_name = req.plan_name
        high_level_plan = req.high_level_plan

        success = True
        if req.task_name == "Excavation":
            loginfo("[Plan Translation Service] translating the high-level plan (" + req.high_level_plan + ") to a PLEXIL plan " + req.plan_name + ".plp for the task, " + req.task_name)
            self.__translation(task_name, plan_name, high_level_plan)
        else:
            loginfo("[Plan Translation Service] Unsupported task: " + req.task_name)
            success = False

        return PlanTranslationResponse(success)
    }



def plan_translation():
    rospy.init_node('plan_translation_node')

    plan_translator = PlanTranslation()

    rospy.loginfo("Ready to provide PLEXIL plan translation service")
    rospy.spin()


if __name__ == "__main__":
    plan_translation()

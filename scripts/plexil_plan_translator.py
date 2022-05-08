#!/usr/bin/env python

from __future__ import print_function

import os
from subprocess import Popen, PIPE

import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import PlanTranslation, PlanTranslationResponse
# PLEXIL plan translation
from plan-translation.plexil_plan_translator import PlexilPlanTranslator

# Environment Variables
# 1 Assist loading run-time info file and dumping synthesized PLEXIL plan to a file
evaluation_root_dir = os.environ("EVALUATION_ROOT_DIR")
# 2 Assist compiling C++-like '#include' headers in synthesized PLEXIL plan (*.plp)
#   ow_simulator : '#include "plan-interface.h"'
#   owlat        : '#include "owlat-interface.h"'
ow_plexil_lib_source_dir = os.environ("OW_PLEXIL_LIB_SOURCE_DIR")
# 3 Assist moving the compiled PLEXIL plan (*.plx) to the destination
plexil_lib_compiled_plan_dir = os.environ("PLEXIL_LIB_COMPILED_PLAN_DIR")

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

        # Step 3 
        # * Compile the PLEXIL plan: plp file to plx file
        # * Move the plx file to the library (<oceanwater_ws>/devel/etc/plexil) of compiled
        #   PLEXIL plans that is inside the devel directory of oceanwater workspace 
        plan_filename = plan_name + ".plp"
        plan_fp = current_plan_dir + "/" + plan_filename 
        loginfo("[Plan Translation - Step 3] Compiling the PLEXIL plan file " + plan_filename)
        cmd_plan_compilation = "plexilc -I " + ow_plexil_lib_source_dir + " -O " + plexil_lib_compiled_plan_dir + plan_fp 
        loginfo("CMD To Run: " + cmd_plan_translation)
        p_pe = Popen(cmd_plan_compilation, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p_pe.communicate()


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

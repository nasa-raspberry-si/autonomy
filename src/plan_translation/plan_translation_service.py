import json
from subprocess import Popen, PIPE

import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import PlanTranslation, PlanTranslationResponse
# PLEXIL plan translation
from plan_translation.plexil_plan_translator import PlexilPlanTranslator


class PlanTranslationService:
    def __init__(self, evaluation_root_dir, ow_plexil_lib_source_dir, plexil_lib_compiled_plan_dir):
        self.evaluation_root_dir = evaluation_root_dir
        self.ow_plexil_lib_source_dir = ow_plexil_lib_source_dir
        self.plexil_lib_compiled_plan_dir = plexil_lib_compiled_plan_dir

        self.translation_service = rospy.Service(
                '/plan_translation',
                PlanTranslation,
                self.callback_plan_translation)
        rospy.loginfo("[Plan Translation Node] service '/plan_translation' is ready")


    def __translation(self, task_name, plan_name, high_level_plan):
        # Step 1
        # * Load runtime info file
        loginfo("[Plan Translation - Step 1] Loading the run-time information for plan translation.")
        task_dir = self.evaluation_root_dir + "/Tasks/" + task_name
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
        cmd_plan_compilation = "plexilc -I " + self.ow_plexil_lib_source_dir + " -O " + self.plexil_lib_compiled_plan_dir + " " + plan_fp 
        loginfo("CMD To Run: " + cmd_plan_compilation)
        p_pe = Popen(cmd_plan_compilation, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p_pe.communicate()


    def callback_plan_translation(self, req):
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

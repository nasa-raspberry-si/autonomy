import os
import json
import random
from subprocess import Popen, PIPE

import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import TaskPlanning, TaskPlanningResponse
# Prism Model Genration
from prism_planning.gen_prism_model import ExcaPrismModelGenerator


class TaskPlanningService:
    def __init__(self, evaluation_root_dir):
        self.evaluation_root_dir = evaluation_root_dir
        # Add additonal items when have a new task for planning
        self.utility_filenames_by_tasks = {
            "Excavation":
            {
                "pp_sh" : "prismpp.sh",
                "preprocessor": "autonomy-excavate.pp",
                "property": "excavate.props",
                "policy_extract" : "PrismPolicy",
                "rt_info": "rt_info.json"
            },
        }

        self.planning_service = rospy.Service(
                '/task_planning',
                TaskPlanning,
                self.callback_task_planning)
        rospy.loginfo("[Task Planning Service] service '/task_planning' is ready")



    # Main steps in the planning
    # 1 Information preparation
    # 2 Generate prism model
    # 3 Query the prism model to get the policy
    # 4 Extract the high-level plan from the policy
    def __planning(self, plan_name, task_name):
        # Step 1
        # * Use the plan_name  to create a directory under the directory,
        #   evaluation_root_dir/TaskX/PlanY, for holding PRISM model, 
        #   policy file and high-level plan file and run-time infor file.
        # * When do the planning, assume that the corresponding run-time info is
        #   ready. That is, the task directory and rt_info.json have been generated.
        # * Meanwhile, the the utility files for running planning, has been created
        #   by the analysis componenet. Check the dict, utility_filenames_by_tasks. 
        loginfo("[Task Planning Service - Step 1] Information Preparation")
        task_dir = self.evaluation_root_dir + "/Tasks/" + task_name
        task_planning_resources_dir = self.evaluation_root_dir + "/Task_Planning_Resources/"
        current_plan_dir = task_dir + "/" + plan_name 
        try:
            if not os.path.exists(current_plan_dir):
                os.makedirs(current_plan_dir)
                loginfo("The directory " + current_plan_dir + " is created")
            else:
                loginfo("The directory " + current_plan_dir + " already exists")
        except OSError as error:
            loginfo(error)

        # * Copy the current runtime info file to the current plan directory
        # * Load runtime information for the following steps
        loginfo("[Task Planning Service - Step 1] Loading the current runtime information...")
        current_rt_info_filepath = task_dir + "/" + self.utility_filenames_by_tasks[task_name]['rt_info']
        cmd = "cp " + current_rt_info_filepath + " " + current_plan_dir
        os.system(cmd)
        runtime_info = {}
        with open(current_rt_info_filepath) as infile:
            runtime_info = json.load(infile)
        loginfo("[Task Planning Service - Step 1] The current runtime information is:")
        loginfo(json.dumps(runtime_info, indent=4, sort_keys=True))

        # Step 2
        # * Generate PRISM model
        loginfo("[Task Planning Service - Step 2]: PRISM model generation")

        # * FIXME: for other task, the handling of runtime info may be different
        num_xlocs = len(runtime_info['xloc_list'])
        num_dlocs = len(runtime_info['dloc_list'])
        prism_model_fp = current_plan_dir + "/exca_"+str(num_xlocs)+"xlocs_"+str(num_dlocs)+"dlocs.prism"

        prism_planning_utility_dir = ""
        if "Excavation" == task_name:
            prism_planing_utility_dir = task_planning_resources_dir + "/Excavation"
        else:
            loginfo("[Task Planning Service - Step 2] Error: unknown task: " + task_name)
        prismpp_sh_fp = prism_planing_utility_dir + "/" + self.utility_filenames_by_tasks[task_name]['pp_sh']
        prism_preprocessor_fp = prism_planing_utility_dir + "/" + self.utility_filenames_by_tasks[task_name]['preprocessor']
        
        # * Generating the Prism model.
        prism_generator = ExcaPrismModelGenerator(
            runtime_info,
            current_plan_dir,
            prismpp_sh_fp,
            prism_model_fp,
            prism_preprocessor_fp)
        prism_generator.generate_prism_model(max_tried = 1)

        # Step 3
        # * Use PRISM model to extract policy
        loginfo("[Task Planning Service - Step 3]: Use PRISM model to extract the policy")
        prism_property_fp = prism_planing_utility_dir + "/" + self.utility_filenames_by_tasks[task_name]['property']
        policy_filename = "policy.adv"
        policy_fp = os.path.join(current_plan_dir, policy_filename)

        cmd_policy_extraction = "prism " + prism_model_fp + \
            " " + prism_property_fp + " -exportadv " + policy_fp
        loginfo("CMD To Run: " + cmd_policy_extraction)
        p_pe = Popen(cmd_policy_extraction, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p_pe.communicate()

        # Step 4
        # * Extract the high-level plan from the Prism policy by using a Java program
        loginfo("[Task Planning Service - Step 4]: Use the policy to generate the high-level plan")

        # * The ending ':' indicates the end of the list of java class paths
        java_class_path_of_PrismPolicy = prism_planing_utility_dir + ":"
        cmd_syn_plan = "java " + \
                "-cp " + java_class_path_of_PrismPolicy + " " + \
                self.utility_filenames_by_tasks[task_name]['policy_extract'] + " " + \
                policy_fp
        loginfo("[Task Planning Service - Step 4] CMD To Run: " + cmd_syn_plan)
        p_sp = Popen(cmd_syn_plan, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p_sp.communicate()
        # high-level plan looks like: "[action1, action2, ...]"
        # the name of each action is prefixed by "select_"
        high_level_plan = stdout.splitlines()[-1]
        # FIXME: some issue in PrismPolicy.java for extracting the high-level plan.
        # Problem:
        #        Sometime, the output by querying a excavation Prism model could
        #        have multiple actions, like "[select_dloc4, select_xloc2, select_dloc4]".
        # Walkround:
        #        As now for demonstrating the functionality of the autonomy, randomly pick
        #        from the high-level plan a excavation location (select_xloc?) if there
        #        are more than one and a dump location (select_dloc?) if there are more
        #        one.
        #        If high-level plan does not contain an excavation location or a dump
        #        location, randomly pick one from the current runtime information. Same
        #        strategy applies to the case when the selected location in the high-level
        #        plan can not be found in the runtime information.
        loginfo("[Task Planning Service - Step 4] high-level plan (raw from Prism model): " + str(high_level_plan))

        # e.g., high_level_plan: "[select_xloc1, select_dloc5]"
        loc_ids = high_level_plan[1:-1].split(", ")
        # determine xloc_id and dloc_id
        xloc_ids = []
        dloc_ids = []
        for loc_id in loc_ids:
            if "xloc" in loc_id:
                xloc_ids.append(loc_id[7:]) # remove the prefix "select_"
            else: # "dloc" is in loc_id
                dloc_ids.append(loc_id[7:])
        # id of selected excavation location
        sel_xloc_id=""
        if len(xloc_ids) != 0:
            if xloc_ids[0] in runtime_info['xloc_list']:
                sel_xloc_id=xloc_ids[0]
        if sel_xloc_id=="": # randomly pick one excavation location from the runtime info
           sel_xloc_id = "xloc" + str(random.randint(1, num_xlocs))
        # id of selected dump location
        sel_dloc_id=""
        if len(dloc_ids) != 0:
            if dloc_ids[0] in runtime_info['dloc_list']:
                sel_dloc_id=dloc_ids[0]
        if sel_dloc_id=="": # randomly pick one excavation location from the runtime info
           sel_dloc_id = "dloc" + str(random.randint(1, num_dlocs))

        # rewrite the high-level plan
        high_level_plan = "[" + "select_"+sel_xloc_id + ", " + "select_"+sel_dloc_id + "]"
        loginfo("[Task Planning Service - Step 4] high-level plan (checked, loc IDs, not names): " + str(high_level_plan))

        # FIXME: location mismatch between automatically generated Prism model and
        #        updated runtime info after removing failed excavation locations
        # Problem:
        #        the current generation of Prism model takes as input, # of excavation
        #        locations and # of dump locations.
        #        For example, numbers of excavation and dump locations are 8 and 4
        #        respectively. It then will generates a Prism model using location ids
        #        from xloc1 to xloc8. Similar setting for dump locations. When a
        #        location is removed, the location ids in the new Prism model will not
        #        not match that in the updated runtime info.
        # Walkaround:
        #        Add an attribute, 'name', to location item. And use the location ids as
        #        keys to retrieve location items in runtime info dictionary for planning
        #        and plan translation.
        xloc_name = runtime_info['xloc_list'][sel_xloc_id]['name']
        dloc_name = runtime_info['dloc_list'][sel_dloc_id]['name']
        loginfo("[Task Planning Service - Step 4] high-level plan (actual loc names): " + xloc_name + ", " + dloc_name)


        high_level_plan_fp = current_plan_dir + "/high_level_plan.txt"
        with open(high_level_plan_fp, "w") as outfile:
            outfile.write(high_level_plan)

        return str(high_level_plan)

    def callback_task_planning(self, req):
        success = True
        high_level_plan_str = ""
        if req.task_name == "Excavation":
            loginfo("[Task Planning Service] synthesizing a plan for the task " + req.task_name)
            high_level_plan_str = self.__planning(req.plan_name, req.task_name)
        else:
            loginfo("[Task Planning Service] Unsupported task: " + req.task_name)
            success = False

        return TaskPlanningResponse(success, high_level_plan_str)


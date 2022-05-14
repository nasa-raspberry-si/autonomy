import os
import json
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
        rospy.loginfo("[Task Planning Node] service '/task_planning' is ready")



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
        loginfo("[Task Planning - Step 1] Information Preparation")
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
        loginfo("[Task Planning - Step 1] Loading the current runtime information...")
        current_rt_info_filepath = task_dir + "/" + self.utility_filenames_by_tasks[task_name]['rt_info']
        cmd = "cp " + current_rt_info_filepath + " " + current_plan_dir
        os.system(cmd)
        runtime_info = {}
        with open(current_rt_info_filepath) as infile:
            runtime_info = json.load(infile)
        loginfo("[Task Planning - Step 1] The current runtime information is:")
        loginfo(json.dumps(runtime_info, indent=4, sort_keys=True))

        # Step 2
        # * Generate PRISM model
        loginfo("[Task Planning - Step 2]: PRISM model generation")

        # * FIXME: for other task, the handling of runtime info may be different
        num_xlocs = len(runtime_info['xloc_list'])
        num_dlocs = len(runtime_info['dloc_list'])
        prism_model_fp = current_plan_dir + "/exca_"+str(num_xlocs)+"xlocs_"+str(num_dlocs)+"dlocs.prism"

        prism_planning_utility_dir = ""
        if "Excavation" == task_name:
            prism_planing_utility_dir = task_planning_resources_dir + "/Excavation"
        else:
            loginfo("[Task Planning - Step 2] Error: unknown task: " + task_name)
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
        loginfo("[Task Planning - Step 3]: Use PRISM model to extract the policy")
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
        loginfo("[Task Planning - Step 4]: Use the policy to generate the high-level plan")

        # * The ending ':' indicates the end of the list of java class paths
        java_class_path_of_PrismPolicy = prism_planing_utility_dir + ":"
        cmd_syn_plan = "java " + \
                "-cp " + java_class_path_of_PrismPolicy + " " + \
                self.utility_filenames_by_tasks[task_name]['policy_extract'] + " " + \
                policy_fp
        loginfo("CMD To Run: " + cmd_syn_plan)
        p_sp = Popen(cmd_syn_plan, shell=True, stdout=PIPE, stderr=PIPE)
        stdout, stderr = p_sp.communicate()
        # high-level plan looks like: "[action1, action2, ...]"
        # the name of each action is prefixed by "select_"
        high_level_plan = stdout.splitlines()[-1]
        loginfo("high-level plan: " + str(high_level_plan))
        high_level_plan_fp = current_plan_dir + "/high_level_plan.txt"
        with open(high_level_plan_fp, "w") as outfile:
            outfile.write(high_level_plan)

        return str(high_level_plan)

    def callback_task_planning(self, req):
        success = True
        high_level_plan_str = ""
        if req.task_name == "Excavation":
            loginfo("[Planning Service] synthesizing a plan for the task " + req.task_name)
            high_level_plan_str = self.__planning(req.plan_name, req.task_name)
        else:
            loginfo("[Planning Service] Unsupported task: " + req.task_name)
            success = False

        return TaskPlanningResponse(success, high_level_plan_str)


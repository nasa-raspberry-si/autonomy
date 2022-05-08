#!/usr/bin/env python

from __future__ import print_function

import os
import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import RTInfoMaintenanceInstruction, RTInfoMaintenanceInstructionResponse
from rs_autonomy.srv import ModelUpdateInstruction, ModelUpdateInstructionResponse

# knowledge_maintenance_tool package
from knowledge_update.maintain_runtime_info import RuntimeInfoMaintenance
from knowledge_update.maintain_models import ModelUpdater


# Environment Variables
# 1 Assist loading run-time info file and dumping synthesized PLEXIL plan to a file
evaluation_root_dir = os.environ("EVALUATION_ROOT_DIR")

# Partially initialize the instance
# Wait for the initial requst for a task to fully initialize it
knowledge_maintenance = RuntimeInfoMaintenance()
knowledge_maintenance.eval_root_dir = evaluation_root_dir
is_km_fully_initialized = False

##################
# Runtime Info Update Options
## 1. Generate: (use all models to generate a new runtime info)
## 2. Update: model_name (SciVal or ExcaProb)
## 3. Remove: location (excavation location or dump location)
##################
# FIXME: currently only consider the excavation scenario  
def maintain_runtime_info(req):
    loginfo("Progressing the runtime maintaince request")
    global knowledge_maintenance
    global is_km_fully_initialized

    result = True

    if req.task_name != "": 
        # fully initialize knowledge_maintenance before any operations
        if !is_km_fully_initialized:
            knowledge_maintenance.task_name = req.task_name
            knowledge_maintenance.extra_initialization()
            is_km_fully_initialized = True

        # Parse request message
        if req.action=="Initialize":
            temp = req.aux_info.split(",")
            # FIXME: currently only consider the excavation scenario
            num_xloc, num_dloc = temp[0], temp[1]
            result = knowledge_maintenance.initialize_rt_info(num_xloc, num_dloc)
        elif req.action=="Update":
            model_names = req.aux_info.split[","]
            result = knowledge_maintenance.update_rt_info(model_names)
        elif req.action=="Remove":
            items_to_remove = req.aux_info.split[","]
            result = knowledge_maintenance.remove_loc(items_to_remove)
        else:
            loginfo("Unknown update action for runtime info: {}".format(argv[1])) 
            loginfo("Currently only support update actions:\n\t{}\n\t{}\n\t{}".format(
                "Initialize <#xloc,#dloc>", "Update [model_names]", "Remove [location IDs]"))
            result = False
    else:
        result = False

    return RTInfoMaintenanceInstructionResponse(result)

def update_models(req):
    loginfo("Progressing the model update request")
    global knowledge_maintenance
    global is_km_fully_initialized

    result = True

    if req.task_name != "": 
        # fully initialize knowledge_maintenance before any operations
        if !is_km_fully_initialized:
            knowledge_maintenance.task_name = req.task_name
            knowledge_maintenance.extra_initialization()
            is_km_fully_initialized = True

        # Updating models
        result = knowledge_maintenance.model_updater.updateModels(model_names)
    else:
        result = False

    return ModelUpdateInstructionResponse(result)


def knowledge_maintainence():
    rospy.init_node('knowledge_node')

    rtInfo_service = rospy.Service(
            '/runtime_info_maintenance',
            RTInfoMaintenanceInstruction,
            maintain_runtime_info)

    modelUpdate_service = rospy.Service(
            '/update_models',
            ModelUpdateInstruction,
            update_models)

    rospy.loginfo("Ready to conduct knowledge maintainence")
    rospy.spin()


if __name__ == "__main__":
    knowledge_maintainence()

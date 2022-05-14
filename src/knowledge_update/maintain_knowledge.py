import rospy
from rospy import loginfo

# ROS services
from rs_autonomy.srv import RTInfoMaintenanceInstruction, RTInfoMaintenanceInstructionResponse
from rs_autonomy.srv import ModelUpdateInstruction, ModelUpdateInstructionResponse

# knowledge-update package
from knowledge_update.maintain_runtime_info import RuntimeInfoMaintenance
from knowledge_update.maintain_models import ModelUpdater


class KnowledgeManagementService:
    def __init__(self, eval_root_dir):
        # Partially initialize the instance
        # Wait for the initial requst for a task to fully initialize it
        self.is_fully_initialized = False
        self.eval_root_dir = eval_root_dir
        self.rfInfoMaintainer = RuntimeInfoMaintenance(eval_root_dir=self.eval_root_dir)
        loginfo("[Knowledge Node] runtime infomation maintainer is partially initialized")

        self.rtInfo_service = rospy.Service(
            '/runtime_info_maintenance',
            RTInfoMaintenanceInstruction,
            self.maintain_runtime_info)
        loginfo("[Knowledge Node] service '/runtime_info_maintenance' is ready")

        self.modelUpdate_service = rospy.Service(
            '/update_models',
            ModelUpdateInstruction,
            self.update_models)
        rospy.loginfo("[Knowledge Node] service '/update_models' is ready")


    ##################
    # Runtime Info Update Options
    ## 1. Generate: (use all models to generate a new runtime info)
    ## 2. Update: model_name (SciVal or ExcaProb)
    ## 3. Remove: location (excavation location or dump location)
    ##################
    # FIXME: currently only consider the excavation scenario  
    def maintain_runtime_info(self, req):
        loginfo("Progressing the runtime maintaince request")
        
        result = True
        
        if req.task_name != "": 
            # fully initialize knowledge_maintenance before any operations
            if not self.is_fully_initialized:
                self.rfInfoMaintainer.task_name = req.task_name
                self.rfInfoMaintainer.extra_initialization()
                self.is_fully_initialized = True

            # Parse request message
            if req.action=="Initialize":
                temp = req.aux_info.split(",")
                # FIXME: currently only consider the excavation scenario
                num_xloc, num_dloc = int(temp[0]), int(temp[1])
                result = self.rfInfoMaintainer.initialize_rt_info(num_xloc, num_dloc)
            elif req.action=="Update":
                model_names = req.aux_info.split[","]
                result = self.rfInfoMaintainer.update_rt_info(model_names)
            elif req.action=="Remove":
                items_to_remove = req.aux_info.split[","]
                result = self.rfInfoMaintainer.remove_loc(items_to_remove)
            else:
                loginfo("Unknown update action for runtime info: {}".format(argv[1])) 
                loginfo("Currently only support update actions:\n\t{}\n\t{}\n\t{}".format(
                    "Initialize <#xloc,#dloc>", "Update [model_names]", "Remove [location IDs]"))
                result = False
        else:
            result = False

        return RTInfoMaintenanceInstructionResponse(result)

    def update_models(self, req):
        loginfo("Progressing the model update request")

        result = True

        if req.task_name != "": 
            # fully initialize knowledge_maintenance before any operations
            if not self.is_fully_initialized:
                self.rfInfoMaintainer.task_name = req.task_name
                self.rfInfoMaintainer.extra_initialization()
                self.is_fully_initialized = True

            # Updating models
            result = self.rfInfoMaintainer.model_updater.updateModels(req.model_names)
        else:
            result = False

        return ModelUpdateInstructionResponse(result)

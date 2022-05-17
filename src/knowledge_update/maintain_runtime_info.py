'''
Create runtime information for excavation as follows:
    1)  generate a pool of locations (2d coordinates) based on the arm's reachable area.
        Please refer to TODO for the arm's reachable area.
    2)  given numbers of excavation and dump locations, randomly draw from the pool.
    3)  for each excavation location, randomly draw a number in [0, 1] as its science
        value and make its excavatability probability inversely proportional to its
        science value to make the planning problem interesting.
'''

from rospy import loginfo
from knowledge_update.maintain_models import ModelUpdater

import random
import json

# RT is short for Runtime
# Each update of runtime info will overwrite the rt_info.json file
# in the directory, EVALUATION_ROOT_DIR/Tasks/TaskName
# When a planning is triggered, the planner, the user of rt_info.json
# should copy rt_info.json to the directory of the current plan
class RuntimeInfoMaintenance():
    def __init__(
            self, task_name="", eval_root_dir="", model_names=["SciVal", "ExcaProb"],
            debug=True):
        self.debug = debug
        self.task_name = task_name
        self.model_names = model_names
        self.loc_pool = self.gen_loc_pool()
        self.eval_root_dir=eval_root_dir
        self.runtime_info = {}
        self.runtime_info_fp = ""
        self.model_updater = None

        self.extra_initialization()

    # API for use
    # Assist the knowledge_maintaince ROS service
    def extra_initialization(self):
        if (self.task_name!="") and (self.eval_root_dir!=""):
            # Check the doc at, doc/structure_of_evaluation_directory.txt
            self.runtime_info_fp = self.eval_root_dir + "/Tasks/" + self.task_name + "/rt_info.json" 
            models_dir = self.eval_root_dir + "/Models"
            self.model_updater = ModelUpdater(models_dir, self.model_names)
            loginfo("The RuntimeInfoMaintenence instance is fully initialized")
        else:
            loginfo("The RuntimeInfoMaintenence instance is not fully initialized due to")
            loginfo("\ttask_name: " + self.task_name)
            loginfo("\teval_root_dir: " + self.eval_root_dir)
            loginfo("\tmodel_names: " + str(self.model_names))


    # FIXME:
    # * The following implementation is for excavation scenario in ow_simulator
    # * Randomly some numbers of locations as candidate excavation and
    #   dump location from a reachable area (Issue 117 in ow_simulator repo).
    #   Issue 117: https://github.com/nasa/ow_simulator/issues/117
    # * The input paramters, x_gap and y_gap, controls the density coverage 
    def gen_loc_pool(self, x_gap=0.2, y_gap=0.1):
        loc_pool = []
        y_steps = int(2/y_gap)
        for y_step in range(0, y_steps+1):
            y = round(1 - y_gap*y_step, 4)
            y_abs = abs(y)
            x_low = 1 + (0.45 + (0.55-0.45)*(1-y_abs))
            x_up  = 1 + (1.2 - 0.6*y_abs)
            x_steps = int((x_up - x_low) / x_gap)
            for x_step in range(0, x_steps+1):
                x = round(x_low + x_gap*x_step, 4)
                loc_pool.append((x, y))

        if self.debug:
            loc_pool.sort(key = lambda x: x[1])
            loginfo("Number of locations in the pool: " + str(len(loc_pool)))
            loginfo(str(loc_pool))

        return loc_pool

    # Science value (SV) and excavatiability probaility
    # * Science value at each location is randomly drawn from (0,1)
    # * To make the planning problem interesting, we generate
    #   execavation probability of each location inversely propotional
    #   to its science value.
    def getSVs(self, num_of_locs):
        sci_vals = []
        for i in range(num_of_locs):
            sci_vals.append(round(random.random(), 4))
        return sci_vals

    def getExProb(self, sv):
        beta = self.model_updater.ExcaProb_beta
        if isinstance(sv, list):
            return [round(prob, 4) for prob in (1-beta*sv)]
        else:
            return round(1-beta*sv, 4)
       
    # API for user
    # FIXME: currently only consider the excavation scenario
    def initialize_rt_info(self, xloc_num=10, dloc_num=6):
        loginfo("Initializing runtime infor for excavation scenario")
        success = True

        # Randomly draw from the sampled reachable area a number of locations
        # as the candidate excavation and dump locations.
        locs = random.sample(self.loc_pool, xloc_num+dloc_num)
        xloc_list = locs[:xloc_num]
        dloc_list = locs[xloc_num:]

        if self.debug:
            loginfo("List of " + str(xloc_num) + " excavation locations:")
            loginfo(xloc_list)
            loginfo("List of " + str(dloc_num) + " dump locations:")
            loginfo(dloc_list)
            '''
            loginfo("\nCreating a scatter plot for excavation and dump locations.\n")
            loc_x_plt = [loc[0] for loc in xloc_list]
            loc_y_plt = [loc[1] for loc in xloc_list]
            plt.scatter(loc_x_plt, loc_y_plt, c="coral", label="Exca Loc")
            loc_x_plt = [loc[0] for loc in dloc_list]
            loc_y_plt = [loc[1] for loc in dloc_list]
            plt.scatter(loc_x_plt, loc_y_plt, c="lightblue", label="Dump Loc")
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title("The spread of excavation and dump locations.")
            plt.legend()
            scatter_plot_fp = runtime_info_fp[:-4] + "jpg"
            plt.savefig(scatter_plot_fp)
            '''

        self.runtime_info = {"task_name": self.task_name, 'xloc_list': {}, 'dloc_list': {}}

        sci_vals = self.getSVs(xloc_num)
        ex_probs = [self.getExProb(sv) for sv in sci_vals]

        for xloc, loc_idx in zip(xloc_list, range(1, xloc_num+1)):
            xloc_ID = "xloc"+str(loc_idx)
            xloc_obj = {
                    "name": xloc_ID,
                    "position": {"x":xloc[0], "y":xloc[1]},
                    "sci_val": sci_vals[loc_idx-1],
                    "ex_prob": ex_probs[loc_idx-1]
                    }
            self.runtime_info["xloc_list"][xloc_ID] = xloc_obj


        for dloc, loc_idx in zip(dloc_list, range(1, dloc_num+1)):
            dloc_ID = "dloc"+str(loc_idx)
            dloc_obj = {
                    "name": dloc_ID,
                    "position": {"x":dloc[0], "y":dloc[1]},
                    }
            self.runtime_info["dloc_list"][dloc_ID] = dloc_obj

        with open(self.runtime_info_fp, "w") as outfile: 
            json.dump(self.runtime_info, outfile)
        loginfo("Runtime info for excavation is ready")

        return success

    # API for user
    def update_rt_info(self, model_names):
        success = True
        if len(model_names) == len(self.model_names):
            xloc_num = len(self.runtime_info['xloc_list'])
            dloc_num = len(self.runtime_info['dloc_list'])
            # re-initialize the runtime info while keeping the numbers of locations
            success = self.initialize_rt_info(xloc_num, dloc_num)
        else:
            res = []
            for model_name in model_names:
                temp_res = self.update_rt_info_per_model(model_name)
                res.append(temp_res)
            if False in res:
                success = False

            with open(self.runtime_info_fp, "w") as outfile: 
                json.dump(self.runtime_info, outfile)
            loginfo("The update of the runtime info is ready")

        return success

    def update_rt_info_per_model(self, model_name):
        success = True
        if model_name == "SciVal":
            self.update_rt_info_usning_SciVal()
        elif model_name == "ExcaProb":
            self.update_rt_info_using_ExcaProb()
        else:
            loginfo("Unknown model: " + model_name)
            msg = "Currently supported models:"
            for model_name in self.model_names:
                msg = msg + "\n\t" + model_name
            loginfo(msg)
            success = False
        return success

    def update_rt_info_using_ExcaProb(self):
        for xloc in self.runtime_info['xloc_list']:
            xloc['ex_prob'] = self.getExProb(xloc['sci_val'])
        

    def update_rt_info_using_SciVal(self):
        xloc_list = self.runtime_info['xloc_list']
        xloc_num = len(xloc_list)
        sci_vals = self.getSVs(num_of_locs)
        # According to current assumption, the excavation probability is
        # inversely propotional to it science value. So, here we need
        # update the excavation probability
        ex_probs = self.getExProb(sci_vals)

        for xloc, idx in zip(xloc_list, range(xloc_num)):
            xloc['sci_val'] = sci_vals[idx]
            xloc['ex_prob'] = ex_probs[idx]
 
    # API for user
    def remove_loc(self, items_to_remove):
        success = True
        for item in items_to_remove:
            if item in self.runtime_info['xloc_list']:
                del self.runtime_info['xloc_list'][item]
            else:
                loginfo("Item ({}) is not an excavation location".format(item))
                success = False

        # Rename location keys in the runtime_info dict due to the naming of 
        # excavation locations in the automatic generation of a PRISM model.
        # If there is 6 excavation locations, the variables' names are:
        # xloc1, xloc2, ..., xloc6
        xloc_list = self.runtime_info['xloc_list']
        self.runtime_info['xloc_list'] = {}
        num_xloc = len(xloc_list)
        xloc_idx = 1
        for key in xloc_list:
            new_key = "xloc" + str(xloc_idx)
            self.runtime_info['xloc_list'][new_key] = xloc_list[key]
            xloc_idx += 1

        with open(self.runtime_info_fp, "w") as outfile: 
            json.dump(self.runtime_info, outfile)
        msg = "Success"
        if not success:
            msg = "Failure"
        loginfo("The update of the runtime info: " + msg)

        return success

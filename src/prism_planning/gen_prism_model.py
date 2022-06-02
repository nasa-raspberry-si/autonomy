import os
from random import seed
from random import random
from random import randint
from math import sqrt

from rospy import loginfo

class ExcaPrismModelGenerator():
    def __init__(self, runtime_info, result_dir, prismpp_sh_fp, prism_model_fp, prism_preprocessor_fp):
        self.rt_info = runtime_info
        self.num_xlocs = len(self.rt_info['xloc_list'])
        self.num_dlocs = len(self.rt_info['dloc_list'])
        self.prismpp_sh_fp = prismpp_sh_fp
        self.prism_preprocessor_fp = prism_preprocessor_fp
        self.prism_model_fp = prism_model_fp

    def generate_skeleton(self):
        cmd = self.prismpp_sh_fp + " " + self.prism_preprocessor_fp + " " + str(self.num_xlocs) + " "+str(self.num_dlocs)+" > "+self.prism_model_fp
        loginfo("[CMD] "+cmd)
        os.system(cmd)

    def cal_dist(self, p1, p2):
        # Euclidean distance.
        # p1 and p2 are points, each of which is dict, {'x': 0.3, 'y': 1}

        return sqrt((p1['x']-p2['x'])**2 + (p1['y']-p2['y'])**2)

    def __generate_move_reward(self, reward_name, reward_factor):
        code = "\n"
	code += "rewards \""+reward_name+"\"\n"
        for s_loc_ID, s_loc in self.rt_info['xloc_list'].items():
            # when the target loc is an excavation loc
            for t_loc_ID, t_loc in self.rt_info['xloc_list'].items():
                if s_loc_ID != t_loc_ID:
                    s_loc_coord = self.rt_info["xloc_list"][s_loc_ID]['position']
                    t_loc_coord = self.rt_info["xloc_list"][t_loc_ID]['position']
                    dist = self.cal_dist(s_loc_coord, t_loc_coord)
                    reward = reward_factor * dist

                    code += "\t[select_"+t_loc_ID+"] loc="+s_loc_ID+" :" + str(reward)+";\n"

            # when the target loc is a dump loc
            for t_loc_ID, t_loc in self.rt_info['dloc_list'].items():
                s_loc_coord = self.rt_info["xloc_list"][s_loc_ID]['position']
                t_loc_coord = self.rt_info["dloc_list"][t_loc_ID]['position']
                dist = self.cal_dist(s_loc_coord, t_loc_coord)
                reward = reward_factor * dist
                code += "\t[select_"+t_loc_ID+"] loc="+s_loc_ID+" :" + str(reward)+";\n"
	code += "endrewards\n"
        return code


    def generate_rewards(self):
	code = ""

	# Generates science value rewards
	code +="// Science value reward\n// The estimated science value for the different excavation locations has to be provided by a different model\n"
	code += "rewards \"SV\"\n"
        for xloc_ID, xloc in self.rt_info['xloc_list'].items():
            code += "\t[select_"+xloc_ID+"] true: "+ str(xloc["sci_val"])+";\n"
	code += "endrewards\n\n"

	# Generates energy consumption rewards
	code += "// Energy consumption cost\n"
	code += "// The values for the energy costs have to be provided by a different model\n"
	code += "// the reward structure below considers both the cost of excavation and moving to the arm to a location\n"
	code += "// the cost of excavation is fixed, but the cost of movement from another location varies, depending on the\n" 
	code += "// original location of the arm (there is one line of the reward structure per alternative original location\n"
	code += "// NOTE: cost of moving the arm A->B and B<-A are the same here, but these costs might be different due to different\n"
	code += "// trajectories computed by lower-level control\n"

	seed(1)
	energy_factor = 1 # random()
        code += self.__generate_move_reward("EC", energy_factor)
	time_factor = 1.0/0.05 # 0.05 represents the average velocity of the arm; random()
        code += self.__generate_move_reward("T", time_factor)

	return code

    def generate_excavatability_probabilities(self):
        code = "\n// Excavatability probabilities for excavation locations\n"
        # e.g., 
        # const double ex_loc1 = 0.6 
        # xloc_ID: xloc2. 
        for xloc_ID, xloc in self.rt_info['xloc_list'].items():
            code += "const double ex_loc"+xloc_ID[4:]+"="+str(xloc['ex_prob'])+";\n"

        return code

    def generate_max_tried(self, max_tried=1):
        code = "\n// Maximum number of excavation attempts (can be 1 by default, number of excavation locations upper bound)\n"
        code += "const int MAX_TRIED="+str(max_tried)+";\n"
        return code

    def generate_prism_model(self, max_tried=1):
        self.generate_skeleton()
        code = ""
        code += self.generate_rewards() + self.generate_excavatability_probabilities() + self.generate_max_tried(max_tried)

	with open(self.prism_model_fp, "a") as myfile:
		myfile.write("\n// Script-generated rewards and constants start here\n\n\n"+code)
	loginfo("A Prism model has been successfully generated.")

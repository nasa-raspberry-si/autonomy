from rospy import loginfo
import json
from math import sqrt

# A scenario represents a task in Europa Lander Mission
class Scenario:
    EXCAVATION = 1
    UNKNOWN = 2
   
    num_str_map = {
            1: "Excavation",
            2: "Unknown"}

    @classmethod
    def get_scenario_str(cls, scenario_num):
        # ToDo
        assert scenario_num <= cls.UNKNOWN
        return cls.num_str_map[scenario_num]


class PlexilPlanTranslator():
    def __init__(self):

        self.scenarios_map = {
                "Excavation": Scenario.EXCAVATION,
                "Unknown": Scenario.UNKNOWN
                }

    # prism_syn_plan: [action1, action2, ...]
    # plexil_plan_name is the name of the plexil node.
    def translate(self, scenario_ID, runtime_info, plexil_plan_name, plexil_plan_dir, prism_syn_plan):
        scenario = self.scenarios_map[scenario_ID]
        # Parse prism_syn_plan
        loginfo("Parsing the synthesized plan by PRISM...")
        plan_info = self.parse_syn_plan(scenario, runtime_info, prism_syn_plan)
        plan_info["node_name"] = plexil_plan_name

        # Generate the PLEXIL code
        loginfo("Generating the PLEXIL code...")
        code = self.generate_code(scenario, plan_info)

        plexil_plan_fp = os.path.join(plexil_plan_dir, plexil_plan_name+".plp") 
        with open(plexil_plan_fp, "w") as outfile:
            outfile.write(code)

    # [Section: Parse Synthesized Plan By PRISM Model]
    def parse_syn_plan(self, scenario, runtime_info, prism_syn_plan):
        plan_info = {}
        if scenario == Scenario.EXCAVATION:
            plan_info = self.parse_syn_plan_exca(runtime_info, prism_syn_plan)
        else:
            loginfo("[Plan Translation] Unknown scenario: " + Scenario.get_scenario_str(scenario))
        return plan_info

    # Excavation
    def parse_syn_plan_exca(self, runtime_info, prism_syn_plan):
        plan_info = {'sel_xloc': {}, 'sel_dloc': {}}

        syn_plan = prism_syn_plan[1:-1].split(', ')
        # Currently only support the case of max_tried=1,
        # which assumes that each action starts with "select_"
        sel_xloc_ID = syn_plan[0][7:]
        sel_dloc_ID = syn_plan[1][7:]
        sel_xloc = runtime_info['xloc_list'][sel_xloc_ID]
        sel_dloc = runtime_info['dloc_list'][sel_dloc_ID]
        loginfo("[Synthesized Excavation Plan]:")
        
        loginfo(">>> selected excavation location: " + sel_xloc["name"] + ":")
        loginfo(json.dumps(sel_xloc, indent=4, sort_keys=True))
        loginfo(">>> selected dump location: " + sel_dloc["name"] + ":")
        loginfo(json.dumps(sel_dloc, indent=4, sort_keys=True))
        
        plan_info['sel_xloc'] = sel_xloc
        plan_info['sel_dloc'] = sel_dloc
        return plan_info


    # [Section: PLEXIL Code Generation]
    def generate_code(self, scenario, plan_info):
        code = ""
        if scenario == Scenario.EXCAVATION:
            code = self.gen_exca_scenario(plan_info)
        else:
            loginfo("[Plan Translation] Unknown scenario: " + scenario_ID)
        return code

    # Excavation
    def gen_exca_scenario(self, plan_info):
        node_name = plan_info["node_name"]
        sel_xloc = plan_info["sel_xloc"]
        sel_dloc = plan_info["sel_dloc"]
        xloc_x = sel_xloc['position']['x']
        xloc_y = sel_xloc['position']['y']
        xloc_sv = sel_xloc['sci_val']
        xloc_ep = sel_xloc['ex_prob']
        dloc_x = sel_dloc['position']['x']
        dloc_y = sel_dloc['position']['y']

        plan_status_varname = "planSuccess"

        loginfo("Generating the PLEXIL code for excavation scenario...")
        code = ""
        code += self.gen_plan_desc()
        code += self.gen_include_files(Scenario.EXCAVATION)
        code += node_name + ":\n"
        code += "{\n"

        # ToDo: implement gen_variables() to handel variable declarations
        code += "\tBoolean " + plan_status_varname + ";\n\n"

        code += self.gen_plan_info(1, plan_info)

        code += self.gen_unstow(1)
        code += self.gen_guarded_move(1, xloc_x, xloc_y)

        msg = "Start the excavation using the grind at " + str(sel_xloc) + "..."
        body_code = self.gen_grind(2, xloc_x, xloc_y, msg=msg)

        body_code_ds = ""
        msg = "Removing tailing...\\n\\tCollecting the tailing using the scoop...\\n"
        body_code_ds += self.gen_dig_circular(3, xloc_x, xloc_y, msg=msg)
        msg = "\\tMoving the tailing to " + str(sel_dloc)
        body_code_ds += self.gen_deliver(3, dloc_x, dloc_y, msg=msg)
           
        msg = "Digging failed."
        # Simulate the status of digging operation
        dig_success_condition = "Lookup(DiggingSuccess(" + str(xloc_ep) + "))"
        body_code += self.gen_dig_success_cond(2, dig_success_condition, plan_status_varname, body_code_ds, msg=msg)
 
        msg = "Skipping grind and dig."
        ground_found_cond = "Lookup(GroundFound)"
        code += self.gen_ground_found_cond(1, body_code, ground_found_cond, plan_status_varname, msg=msg)

        code += self.gen_stow(1)
        update_node_name = "ExcavationStatus"
        update_msg = {}
        # ToDo: handel literals and variables appropriately
        update_msg['plan_name'] = "\""+node_name+"\"" # a string literal in plexil plan
        update_msg['exec_status'] = plan_status_varname # a variable in plexil plan
        code += self.gen_update_node(1, update_node_name, update_msg)
        code += "\tlog_info (\"Excavation is finished.\");\n"
        code += "}\n"

        return code
 
      
    # [Section: Code Generation For Each Primitive Part]
    def gen_plan_desc(self):
        code = "// The following PLEXIL codes are automatically generated during planning.\n\n"
        return code

    def gen_include_files(self, scenario):
        code = "#include \"plan-interface.h\"\n"
        if scenario == Scenario.EXCAVATION:
            pass
        code += "\n\n"
        return code

    def gen_plan_info(self, tab_num, plan_info):
        pre_dents = self.__gen_ident_str(tab_num)
        code = pre_dents + "log_info (\"" + str(plan_info) + "\");\n\n"
        return code

    def gen_unstow(self, tab_num):
        pre_dents = self.__gen_ident_str(tab_num)
        code = ["log_info (\"Unstowing arm...\");\n"]
        code.append("LibraryCall Unstow();\n\n")
        return self.__ident_code(code, pre_dents)

    def gen_stow(self, tab_num):
        pre_dents = self.__gen_ident_str(tab_num)
        code = ["log_info (\"Stowing arm...\");\n"]
        code.append("LibraryCall Stow();\n\n")
        return self.__ident_code(code, pre_dents)

    def gen_guarded_move(self, tab_num, x, y, z=0.05, dir_x=0, dir_y=0, dir_z=1, search_dist=0.25):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        code.append("log_info (\"A guarded move to find out the ground position of the location (x="+str(x)+", y="+str(y)+")...\");\n")
        code.append("LibraryCall GuardedMove (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z) + ",\n")
        code.append("\tDirX = " + str(dir_x) + ", DirY = " + str(dir_y) + ", DirZ = " + str(dir_z) + ",\n")
        code.append("\tSearchDistance = " + str(search_dist) + ");\n\n")

        return self.__ident_code(code, pre_dents)

    def gen_ground_found_cond(self, tab_num, body_code, condition, plan_status_varname, msg=""):
        pre_dents = self.__gen_ident_str(tab_num) 
        code = pre_dents + plan_status_varname + "=" + condition + ";\n"
        code += pre_dents + "if (" + plan_status_varname + ") {\n"
 
        #code = pre_dents + "if (Lookup(GroundFound)) {\n"
        code += body_code
        code += pre_dents + "}\n"

        code += pre_dents + "else log_error (\"Failed to find ground"
        if msg != "":
            code += ": " + msg
        code += "\");\n"
        
        code += pre_dents + "endif\n\n"

        return code

    def gen_dig_success_cond(self, tab_num, condition, plan_status_varname, body_code, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = pre_dents + plan_status_varname + "=" + condition + ";\n"
        code += pre_dents + "if (" + plan_status_varname + ") {\n"
        code += pre_dents + "\tlog_info (\"Digging successed!\");\n"
        code += body_code
        code += pre_dents + "}\n"

        code += pre_dents + "else log_error (\"[Condition Failed] " + condition
        if msg != "":
            code += ": " + msg
        code += "\");\n"
        
        code += pre_dents + "endif\n\n"

        return code

    def gen_grind(self, tab_num, x, y, depth=0.05, length=0.2, parallel=True, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Start grinding...\");\n")
        code.append("LibraryCall Grind (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ", Depth = " + str(depth) + ",\n")
        code.append("\tLength = " + str(length) + ", Parallel = " + self.__get_bool_str(parallel) + ",\n")
        code.append("\tGroundPos = Lookup(GroundPosition));\n\n")

        return self.__ident_code(code, pre_dents)


    def gen_dig_circular(self, tab_num, x, y, depth=0.05, parallel=True, msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Start dig_circular...\");\n")
        code.append("LibraryCall DigCircular (\n")
        code.append("\tX = " + str(x) + ", Y = " + str(y) + ", Depth = " + str(depth) + ",\n")
        code.append("\tGroundPos = Lookup(GroundPosition),\n")
        code.append("\tParallel = " + self.__get_bool_str(parallel) + ");\n\n")
 
        return self.__ident_code(code, pre_dents)

    def gen_deliver(self, tab_num, x, y, z=0.5,  msg=""):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        if msg != "":
            code.append("log_info (\""+ msg + "\");\n")
        else:
            code.append("log_info (\"Delivering the sample...\");\n")
        code.append("LibraryCall Deliver (X = " + str(x) + ", Y = " + str(y) + ", Z = " + str(z) + ");\n\n")
        
        return self.__ident_code(code, pre_dents)

    # Currently only support the Update statement in the Update node
    # Input:
    #   update_msg: a dict of messages, each of which has the format 'variable_name=value' in the Update Statement
    def gen_update_node(self, tab_num, update_node_name, update_msg):
        pre_dents = self.__gen_ident_str(tab_num)
        code = []
        code.append(update_node_name+":\n")
        code.append("{\n")
        cnt = 0
        update_content = "\tUpdate "
        for variable_key in update_msg:
            if cnt != 0:
                update_content += ", "
            update_content += variable_key + "=" + update_msg[variable_key]
            cnt += 1
        update_content += ";\n"
        code.append(update_content)
        code.append("}\n\n")
        
        return self.__ident_code(code, pre_dents)


    # [Section: utility functions]
    #
    # Python and Plexil have different boolean literals
    # Language   Boolean Literals
    # Python     True   |   False
    # Plexil     true   |   false
    def __get_bool_str(self, bool_var):
        if bool_var:
            return "true"
        else:
            return "false"

    def __ident_code(self, code, pre_dents):
        code_str = ""
        for line in code:
            code_str += pre_dents+line
        return code_str

    def __gen_ident_str(self, tab_num):
        return "\t"*tab_num

    def gen_info(self, tab_num, msg=""):
        code = ["log_info (\"" + msg + "\");\n"]
        pre_dents = self.__gen_ident_str(tab_num)
        return self.__ident_code(code, pre_dents)
